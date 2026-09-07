// Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Minimal ESP32-C6 GDMA (General DMA) driver.
//!
//! This driver provides just enough functionality to feed the I2S peripheral
//! with data, since the ESP32-C6 I2S FIFO is only reachable via GDMA. It uses
//! single-descriptor transfers with busy-polling (no interrupt registration).
//!
//! The descriptor format follows the standard ESP32 family layout: a 12-byte
//! (3-word) linked-list node. See the ESP32-C6 Technical Reference Manual.

use core::ptr::addr_of;

use crate::static_ref::StaticRef;
use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};

/// GDMA base address on ESP32-C6.
const DMA_BASE: usize = 0x6008_0000;

/// Per-channel register stride (CH[n] starts at `0x70 + n * 0x80`).
const CH_OFFSET: usize = 0x70;
const CH_STRIDE: usize = 0x80;

/// IN_INT_CH[n] stride: each cluster is 0x10 (RAW, ST, ENA, CLR).
const IN_INT_BASE: usize = 0x00;
const OUT_INT_BASE: usize = 0x30;
const INT_STRIDE: usize = 0x10;

/// Peripheral IDs for `peri_sel` (ESP32-C6).
/// I2S0 = 3, SPI2 = 0, UHCI0 = 2, etc.
pub const PERI_I2S0: u32 = 3;

/// Polling iteration cap before declaring a DMA timeout.
const DMA_POLL_LIMIT: u32 = 10_000_000;

register_bitfields! [
    u32,

    /// RX (input) interrupt bits — shared by RAW / ST / ENA / CLR.
    pub InInt [
        IN_DONE     OFFSET(0) NUMBITS(1) [],
        IN_SUC_EOF  OFFSET(1) NUMBITS(1) [],
        IN_DSCR_ERR OFFSET(3) NUMBITS(1) [],
    ],

    /// TX (output) interrupt bits — shared by RAW / ST / ENA / CLR.
    pub OutInt [
        OUT_DONE      OFFSET(0) NUMBITS(1) [],
        OUT_EOF       OFFSET(1) NUMBITS(1) [],
        OUT_DSCR_ERR  OFFSET(2) NUMBITS(1) [],
        OUT_TOTAL_EOF OFFSET(3) NUMBITS(1) [],
    ],

    /// RX channel configure 0.
    pub InConf0 [
        IN_RST           OFFSET(0) NUMBITS(1) [],
        INDSCR_BURST_EN  OFFSET(2) NUMBITS(1) [],
        IN_DATA_BURST_EN OFFSET(3) NUMBITS(1) [],
        MEM_TRANS_EN     OFFSET(4) NUMBITS(1) [],
    ],

    /// TX channel configure 0.
    pub OutConf0 [
        OUT_RST           OFFSET(0) NUMBITS(1) [],
        OUT_EOF_MODE      OFFSET(3) NUMBITS(1) [],
        OUTDSCR_BURST_EN  OFFSET(4) NUMBITS(1) [],
        OUT_DATA_BURST_EN OFFSET(5) NUMBITS(1) [],
    ],

    /// RX inlink descriptor control.
    pub InLink [
        INLINK_ADDR    OFFSET(0)  NUMBITS(20) [],
        INLINK_START   OFFSET(22) NUMBITS(1) [],
        INLINK_RESTART OFFSET(23) NUMBITS(1) [],
        INLINK_PARK    OFFSET(24) NUMBITS(1) [],
    ],

    /// TX outlink descriptor control.
    pub OutLink [
        OUTLINK_ADDR    OFFSET(0)  NUMBITS(20) [],
        OUTLINK_START   OFFSET(21) NUMBITS(1) [],
        OUTLINK_RESTART OFFSET(22) NUMBITS(1) [],
        OUTLINK_PARK    OFFSET(23) NUMBITS(1) [],
    ],

    /// RX / TX peripheral selection (6-bit ID).
    pub PeriSel [
        PERI_SEL OFFSET(0) NUMBITS(6) [],
    ],
];

register_structs! {
    /// One IN_INT / OUT_INT interrupt cluster (RAW, ST, ENA, CLR).
    ///
    /// The same struct layout is used for both RX (input) and TX (output)
    /// interrupt clusters. The field type (`InInt` or `OutInt`) is selected
    /// by the caller — the raw `u32` register layout is identical.
    pub IntCluster {
        (0x00 => raw: ReadOnly<u32>),
        (0x04 => st: ReadOnly<u32>),
        (0x08 => ena: ReadWrite<u32>),
        (0x0c => clr: WriteOnly<u32>),
        (0x10 => @END),
    }
}

register_structs! {
    /// One DMA channel register block (CH[n]).
    ///
    /// The RX (input) side occupies the first 0x60 bytes; a 0x2c-byte reserved
    /// gap separates it from the TX (output) side which starts at +0x60.
    pub DmaChannel {
        // ---- RX (input: peripheral → memory) ----
        (0x00 => in_conf0: ReadWrite<u32, InConf0::Register>),
        (0x04 => _in_conf1),
        (0x08 => _infifo_status),
        (0x0c => _in_pop),
        (0x10 => in_link: ReadWrite<u32, InLink::Register>),
        (0x14 => _in_state),
        (0x18 => _in_suc_eof_des_addr),
        (0x1c => _in_err_eof_des_addr),
        (0x20 => _in_dscr),
        (0x24 => _in_dscr_bf0),
        (0x28 => _in_dscr_bf1),
        (0x2c => _in_pri),
        (0x30 => in_peri_sel: ReadWrite<u32, PeriSel::Register>),
        (0x34 => _reserved_rx),
        // ---- TX (output: memory → peripheral) ----
        (0x60 => out_conf0: ReadWrite<u32, OutConf0::Register>),
        (0x64 => _out_conf1),
        (0x68 => _outfifo_status),
        (0x6c => _out_push),
        (0x70 => out_link: ReadWrite<u32, OutLink::Register>),
        (0x74 => _out_state),
        (0x78 => _out_eof_des_addr),
        (0x7c => _out_eof_bfr_des_addr),
        (0x80 => _out_dscr),
        (0x84 => _out_dscr_bf0),
        (0x88 => _out_dscr_bf1),
        (0x8c => _out_pri),
        (0x90 => out_peri_sel: ReadWrite<u32, PeriSel::Register>),
        (0x94 => _reserved_end),
        (0xa0 => @END),
    }
}

/// ESP32-C6 GDMA linked-list descriptor (3 words = 12 bytes).
///
/// `dw0` packs: `length[11:0] | size[23:12] | suc_eof[24] | owner[25]`.
/// `buffer` is the data buffer address (word-aligned, internal SRAM).
/// `next` is the next descriptor address, or null for end-of-list.
#[repr(C, align(4))]
pub struct DmaDescriptor {
    pub dw0: u32,
    pub buffer: *mut u8,
    pub next: *mut DmaDescriptor,
}

impl DmaDescriptor {
    /// Build a TX descriptor for `buf`. `suc_eof` marks the last descriptor so
    /// GDMA raises `OUT_TOTAL_EOF` after consuming it.
    pub const fn for_tx(buf: *mut u8, len: usize, suc_eof: bool) -> Self {
        let size = len as u32;
        let eof = if suc_eof { 1 << 24 } else { 0 };
        // owner = 0 (DMA) — the CPU hands the buffer to DMA by clearing owner.
        DmaDescriptor {
            dw0: (size << 12) | eof,
            buffer: buf,
            next: core::ptr::null_mut(),
        }
    }

    /// Build an RX descriptor for `buf`. `size` is the capacity; after the
    /// transfer completes, `length` (bits 11:0 of `dw0`) holds the received
    /// byte count.
    pub const fn for_rx(buf: *mut u8, capacity: usize) -> Self {
        let size = capacity as u32;
        DmaDescriptor {
            dw0: size << 12,
            buffer: buf,
            next: core::ptr::null_mut(),
        }
    }

    /// Received byte count (valid after an RX transfer).
    pub fn received_len(&self) -> usize {
        (self.dw0 & 0xfff) as usize
    }
}

/// Returns a `&'static DmaChannel` for channel `CH`.
fn channel_regs<const CH: usize>() -> &'static DmaChannel {
    assert!(CH < 3, "ESP32-C6 has only 3 GDMA channels");
    let base = DMA_BASE + CH_OFFSET + CH * CH_STRIDE;
    unsafe { &*(base as *const DmaChannel) }
}

/// Returns a `&'static IntCluster` for the RX (input) interrupt block of
/// channel `CH`.
fn in_int_regs<const CH: usize>() -> &'static IntCluster {
    let base = DMA_BASE + IN_INT_BASE + CH * INT_STRIDE;
    unsafe { &*(base as *const IntCluster) }
}

/// Returns a `&'static IntCluster` for the TX (output) interrupt block of
/// channel `CH`.
fn out_int_regs<const CH: usize>() -> &'static IntCluster {
    let base = DMA_BASE + OUT_INT_BASE + CH * INT_STRIDE;
    unsafe { &*(base as *const IntCluster) }
}

fn wait_for_bit(regs: &IntCluster, mask: u32) -> bool {
    for _ in 0..DMA_POLL_LIMIT {
        if regs.raw.get() & mask != 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// ESP32-C6 GDMA channel driver.
///
/// `CH` is the channel index (0, 1, or 2). A single channel can serve either
/// TX or RX for one peripheral — the I2S driver uses two instances (one for
/// playback, one for capture).
///
/// All methods are associated functions (no `&self` state) so the driver can
/// be used from a `static` context.
pub struct Esp32c6GdmaChannel<const CH: usize>;

impl<const CH: usize> Esp32c6GdmaChannel<CH> {
    pub const fn new() -> Self {
        assert!(CH < 3, "ESP32-C6 has only 3 GDMA channels");
        Self {}
    }

    // ---- RX (input: peripheral → memory) ----

    /// Reset the RX FSM and FIFO.
    pub fn reset_rx() {
        let ch = channel_regs::<CH>();
        ch.in_conf0.modify(InConf0::IN_RST::SET);
        ch.in_conf0.modify(InConf0::IN_RST::CLEAR);
    }

    /// Select which peripheral feeds this RX channel (`peri_sel`).
    pub fn set_rx_peri(peri: u32) {
        let ch = channel_regs::<CH>();
        ch.in_peri_sel.write(PeriSel::PERI_SEL.val(peri & 0x3f));
    }

    /// Start an RX transfer using `desc` as the inlink descriptor. The
    /// descriptor (and its buffer) must remain valid until the transfer
    /// completes.
    pub fn start_rx(desc: &DmaDescriptor) {
        Self::reset_rx();
        // Clear any pending RX interrupt status.
        let int = in_int_regs::<CH>();
        int.clr.set(InInt::IN_DONE.val(1).value | InInt::IN_SUC_EOF.val(1).value | InInt::IN_DSCR_ERR.val(1).value);
        // Write the descriptor address into INLINK_ADDR and kick off.
        let ch = channel_regs::<CH>();
        let desc_addr = (addr_of!(*desc) as usize as u32) & ((1 << 20) - 1);
        ch.in_link.write(InLink::INLINK_ADDR.val(desc_addr) + InLink::INLINK_RESTART::SET);
        ch.in_link.write(InLink::INLINK_ADDR.val(desc_addr) + InLink::INLINK_START::SET);
    }

    /// Returns `true` once the RX transfer has completed (`IN_SUC_EOF`).
    pub fn is_rx_done() -> bool {
        in_int_regs::<CH>().raw.get() & InInt::IN_SUC_EOF.mask != 0
    }

    /// Block until the RX transfer completes. Returns `Err` on timeout or
    /// descriptor error.
    pub fn wait_rx_done() -> blueos_hal::err::Result<()> {
        let int = in_int_regs::<CH>();
        loop {
            let raw = int.raw.get();
            if raw & InInt::IN_SUC_EOF.mask != 0 {
                return Ok(());
            }
            if raw & InInt::IN_DSCR_ERR.mask != 0 {
                return Err(blueos_hal::err::HalError::Fail);
            }
            if !wait_for_bit(int, InInt::IN_SUC_EOF.mask | InInt::IN_DSCR_ERR.mask) {
                return Err(blueos_hal::err::HalError::Timeout);
            }
        }
    }

    // ---- TX (output: memory → peripheral) ----

    /// Reset the TX FSM and FIFO.
    pub fn reset_tx() {
        let ch = channel_regs::<CH>();
        ch.out_conf0.modify(OutConf0::OUT_RST::SET);
        ch.out_conf0.modify(OutConf0::OUT_RST::CLEAR);
    }

    /// Select which peripheral this TX channel feeds (`peri_sel`).
    pub fn set_tx_peri(peri: u32) {
        let ch = channel_regs::<CH>();
        ch.out_peri_sel.write(PeriSel::PERI_SEL.val(peri & 0x3f));
    }

    /// Start a TX transfer using `desc` as the outlink descriptor. The
    /// descriptor (and its buffer) must remain valid until the transfer
    /// completes.
    pub fn start_tx(desc: &DmaDescriptor) {
        Self::reset_tx();
        // Clear any pending TX interrupt status.
        let int = out_int_regs::<CH>();
        int.clr.set(OutInt::OUT_DONE.val(1).value | OutInt::OUT_EOF.val(1).value | OutInt::OUT_TOTAL_EOF.val(1).value | OutInt::OUT_DSCR_ERR.val(1).value);
        let ch = channel_regs::<CH>();
        let desc_addr = (addr_of!(*desc) as usize as u32) & ((1 << 20) - 1);
        ch.out_link.write(OutLink::OUTLINK_ADDR.val(desc_addr) + OutLink::OUTLINK_RESTART::SET);
        ch.out_link.write(OutLink::OUTLINK_ADDR.val(desc_addr) + OutLink::OUTLINK_START::SET);
    }

    /// Returns `true` once the TX transfer has completed (`OUT_TOTAL_EOF`).
    pub fn is_tx_done() -> bool {
        out_int_regs::<CH>().raw.get() & OutInt::OUT_TOTAL_EOF.mask != 0
    }

    /// Block until the TX transfer completes. Returns `Err` on timeout or
    /// descriptor error.
    pub fn wait_tx_done() -> blueos_hal::err::Result<()> {
        let int = out_int_regs::<CH>();
        loop {
            let raw = int.raw.get();
            if raw & OutInt::OUT_TOTAL_EOF.mask != 0 {
                return Ok(());
            }
            if raw & OutInt::OUT_DSCR_ERR.mask != 0 {
                return Err(blueos_hal::err::HalError::Fail);
            }
            if !wait_for_bit(int, OutInt::OUT_TOTAL_EOF.mask | OutInt::OUT_DSCR_ERR.mask) {
                return Err(blueos_hal::err::HalError::Timeout);
            }
        }
    }
}

unsafe impl<const CH: usize> Send for Esp32c6GdmaChannel<CH> {}
unsafe impl<const CH: usize> Sync for Esp32c6GdmaChannel<CH> {}
