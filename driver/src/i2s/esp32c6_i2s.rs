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

//! ESP32-C6 I2S0 register-level driver.
//!
//! The ESP32-C6 I2S peripheral has no APB-visible data FIFO; all sample data
//! must flow through GDMA. This driver pairs the I2S0 register block with two
//! GDMA channels (one TX, one RX) and exposes the `blueos_hal::i2s::I2s` trait.
//!
//! The clock tree is: XTAL (40 MHz) → PCR.I2S_TX_CLKM_DIV → I2S core clock.
//! For MCLK = 256×fs the divider is `40_000_000 / (256 * sample_rate)`. The
//! I2S block then derives BCK = MCLK / (2 * half_sample_bits).

use blueos_hal::i2s::{I2s, I2sChannelMode, I2sConfig, I2sFormat};
use blueos_hal::{Configuration, PlatPeri};
use core::cell::UnsafeCell;

use crate::dma::esp32c6_gdma::{DmaDescriptor, Esp32c6GdmaChannel, PERI_I2S0};
use crate::static_ref::StaticRef;
use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};

/// I2S0 peripheral base address on ESP32-C6.
const I2S0_BASE: usize = 0x6000_c000;
/// PCR (Peripheral Clock Reset) base address on ESP32-C6.
const PCR_BASE: usize = 0x6009_6000;
/// XTAL frequency on ESP32-C6 (40 MHz).
const XTAL_HZ: u32 = 40_000_000;

/// Polling iteration cap for DMA wait.
const POLL_LIMIT: u32 = 10_000_000;

register_bitfields! [
    u32,

    /// I2S interrupt bits — shared by INT_RAW / INT_ST / INT_ENA / INT_CLR.
    pub I2sInt [
        RX_DONE OFFSET(0) NUMBITS(1) [],
        TX_DONE OFFSET(1) NUMBITS(1) [],
        RX_HUNG  OFFSET(2) NUMBITS(1) [],
        TX_HUNG  OFFSET(3) NUMBITS(1) [],
    ],

    /// I2S TX configuration register.
    pub TxConf [
        TX_RESET       OFFSET(0)  NUMBITS(1) [],
        TX_FIFO_RESET  OFFSET(1)  NUMBITS(1) [],
        TX_START       OFFSET(2)  NUMBITS(1) [],
        TX_SLAVE_MOD   OFFSET(3)  NUMBITS(1) [],
        TX_MONO        OFFSET(5)  NUMBITS(1) [],
        TX_CHAN_EQUAL  OFFSET(6)  NUMBITS(1) [],
        TX_UPDATE       OFFSET(8)  NUMBITS(1) [],
        TX_PCM_BYPASS  OFFSET(12) NUMBITS(1) [],
        TX_STOP_EN     OFFSET(13) NUMBITS(1) [],
        TX_TDM_EN      OFFSET(19) NUMBITS(1) [],
        TX_PDM_EN      OFFSET(20) NUMBITS(1) [],
        TX_CHAN_MOD    OFFSET(24) NUMBITS(3) [],
        SIG_LOOPBACK   OFFSET(27) NUMBITS(1) [],
    ],

    /// I2S RX configuration register.
    pub RxConf [
        RX_RESET       OFFSET(0)  NUMBITS(1) [],
        RX_FIFO_RESET  OFFSET(1)  NUMBITS(1) [],
        RX_START       OFFSET(2)  NUMBITS(1) [],
        RX_SLAVE_MOD   OFFSET(3)  NUMBITS(1) [],
        RX_MONO        OFFSET(5)  NUMBITS(1) [],
        RX_UPDATE       OFFSET(8)  NUMBITS(1) [],
        RX_PCM_BYPASS  OFFSET(12) NUMBITS(1) [],
        RX_TDM_EN      OFFSET(19) NUMBITS(1) [],
        RX_PDM_EN      OFFSET(20) NUMBITS(1) [],
    ],

    /// I2S TX_CONF1 / RX_CONF1 — bit-clock divider, data width, half-sample
    /// bits, TDM channel bits, and MSB-shift (Philips standard).
    pub Conf1 [
        TDM_WS_WIDTH      OFFSET(0)  NUMBITS(7)  [],
        BCK_DIV_NUM       OFFSET(7)  NUMBITS(6)  [],
        BITS_MOD          OFFSET(13) NUMBITS(5)  [],
        HALF_SAMPLE_BITS  OFFSET(18) NUMBITS(6)  [],
        TDM_CHAN_BITS     OFFSET(24) NUMBITS(5)  [],
        MSB_SHIFT         OFFSET(29) NUMBITS(1)  [],
    ],

    /// I2S TX/RX TDM control register.
    pub TdmCtrl [
        TDM_CHAN_EN    OFFSET(0)  NUMBITS(16) [],
        TDM_TOT_CHAN_NUM OFFSET(16) NUMBITS(4) [],
    ],

    /// I2S state register (read-only).
    pub I2sState [
        TX_IDLE OFFSET(0) NUMBITS(1) [],
    ],

    /// PCR.I2S_CONF — APB clock enable and module reset.
    pub PcrI2sConf [
        I2S_CLK_EN OFFSET(0) NUMBITS(1) [],
        I2S_RST_EN OFFSET(1) NUMBITS(1) [],
    ],

    /// PCR.I2S_TX_CLKM_CONF / PCR.I2S_RX_CLKM_CONF — function clock source
    /// and divider. The RX variant has an extra MCLK_SEL bit at offset 23.
    pub PcrI2sClkmConf [
        I2S_CLKM_DIV_NUM OFFSET(12) NUMBITS(8)  [],
        I2S_CLKM_SEL     OFFSET(20) NUMBITS(2)  [],
        I2S_CLKM_EN      OFFSET(22) NUMBITS(1)  [],
        I2S_MCLK_SEL     OFFSET(23) NUMBITS(1)  [],
    ],
];

register_structs! {
    /// I2S0 register block.
    pub I2sRegisters {        (0x00 => _reserved0),
        (0x0c => int_raw: ReadOnly<u32, I2sInt::Register>),
        (0x10 => int_st: ReadOnly<u32, I2sInt::Register>),
        (0x14 => int_ena: ReadWrite<u32, I2sInt::Register>),
        (0x18 => int_clr: WriteOnly<u32, I2sInt::Register>),
        (0x1c => _reserved1),
        (0x20 => rx_conf: ReadWrite<u32, RxConf::Register>),
        (0x24 => tx_conf: ReadWrite<u32, TxConf::Register>),
        (0x28 => rx_conf1: ReadWrite<u32, Conf1::Register>),
        (0x2c => tx_conf1: ReadWrite<u32, Conf1::Register>),
        (0x30 => _rx_clkm_conf),
        (0x34 => _tx_clkm_conf),
        (0x38 => _rx_clkm_div_conf),
        (0x3c => _tx_clkm_div_conf),
        (0x40 => _tx_pcm2pdm_conf),
        (0x44 => _tx_pcm2pdm_conf1),
        (0x48 => _reserved2),
        (0x50 => rx_tdm_ctrl: ReadWrite<u32, TdmCtrl::Register>),
        (0x54 => tx_tdm_ctrl: ReadWrite<u32, TdmCtrl::Register>),
        (0x58 => _rx_timing),
        (0x5c => _tx_timing),
        (0x60 => lc_hung_conf: ReadWrite<u32>),
        (0x64 => rxeof_num: ReadWrite<u32>),
        (0x68 => _conf_sigle_data),
        (0x6c => state: ReadOnly<u32, I2sState::Register>),
        (0x70 => _etm_conf),
        (0x74 => _reserved3),
        (0x80 => date: ReadWrite<u32>),
        (0x84 => @END),
    }
}

register_structs! {
    /// PCR I2S register block (offsets 0x6c..0x80 within PCR).
    pub PcrI2sRegisters {
        (0x00 => _reserved0),
        (0x6c => i2s_conf: ReadWrite<u32, PcrI2sConf::Register>),
        (0x70 => i2s_tx_clkm_conf: ReadWrite<u32, PcrI2sClkmConf::Register>),
        (0x74 => _i2s_tx_clkm_div_conf),
        (0x78 => i2s_rx_clkm_conf: ReadWrite<u32, PcrI2sClkmConf::Register>),
        (0x7c => _i2s_rx_clkm_div_conf),
        (0x80 => @END),
    }
}

/// ESP32-C6 I2S0 driver with two GDMA channels.
///
/// `TX_CH` and `RX_CH` are GDMA channel indices (0, 1, or 2). They must be
/// distinct if both playback and capture are used simultaneously.
///
/// The driver stores a TX descriptor and RX descriptor internally (single
/// descriptor per transfer — callers feed buffers chunked to a reasonable
/// size). The buffers themselves are supplied by the caller and must outlive
/// the transfer.
pub struct Esp32c6I2s0<const TX_CH: usize, const RX_CH: usize> {
    registers: StaticRef<I2sRegisters>,
    pcr: StaticRef<PcrI2sRegisters>,
    tx_desc: UnsafeCell<DmaDescriptor>,
    rx_desc: UnsafeCell<DmaDescriptor>,
}

unsafe impl<const TX_CH: usize, const RX_CH: usize> Send for Esp32c6I2s0<TX_CH, RX_CH> {}
unsafe impl<const TX_CH: usize, const RX_CH: usize> Sync for Esp32c6I2s0<TX_CH, RX_CH> {}

impl<const TX_CH: usize, const RX_CH: usize> Esp32c6I2s0<TX_CH, RX_CH> {
    pub const fn new() -> Self {
        assert!(TX_CH < 3, "ESP32-C6 has only 3 GDMA channels");
        assert!(RX_CH < 3, "ESP32-C6 has only 3 GDMA channels");
        Self {
            registers: unsafe { StaticRef::new(I2S0_BASE as *const I2sRegisters) },
            pcr: unsafe { StaticRef::new(PCR_BASE as *const PcrI2sRegisters) },
            tx_desc: UnsafeCell::new(DmaDescriptor {
                dw0: 0,
                buffer: core::ptr::null_mut(),
                next: core::ptr::null_mut(),
            }),
            rx_desc: UnsafeCell::new(DmaDescriptor {
                dw0: 0,
                buffer: core::ptr::null_mut(),
                next: core::ptr::null_mut(),
            }),
        }
    }

    /// Configure MCLK and BCK dividers for the given sample rate / bits / channels.
    ///
    /// MCLK = 256 × fs (standard for ES8311). The PCR divider turns the 40 MHz
    /// XTAL into MCLK: `div = XTAL / MCLK`. The I2S BCK divider then produces
    /// BCK = MCLK / (2 × half_sample_bits), where `half_sample_bits = bits ×
    /// channels / 2`.
    fn configure_clock(&self, sample_rate: u32, bits: u8, channels: u8) -> blueos_hal::err::Result<()> {
        if sample_rate == 0 || bits == 0 || channels == 0 {
            return Err(blueos_hal::err::HalError::InvalidParam);
        }

        // MCLK = 256 × fs.
        let mclk = sample_rate * 256;
        if mclk > XTAL_HZ {
            return Err(blueos_hal::err::HalError::NotSupport);
        }
        let mclk_div = XTAL_HZ / mclk;
        if mclk_div == 0 || mclk_div > 255 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }

        // half_sample_bits = (bits × channels) / 2. For 16-bit stereo = 16.
        let half_sample_bits = (bits as u32 * channels as u32) / 2;
        if half_sample_bits == 0 || half_sample_bits > 63 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }

        // BCK = sample_rate * bits * channels.
        let bck_target = sample_rate * bits as u32 * channels as u32;
        let bck_div_num = mclk / bck_target;
        if bck_div_num == 0 || bck_div_num > 64 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }
        let bck_div_field = bck_div_num.saturating_sub(1) & 0x3f;

        // TX clock: select XTAL (0), set divider, enable.
        self.pcr.i2s_tx_clkm_conf.modify(
            PcrI2sClkmConf::I2S_CLKM_DIV_NUM.val(mclk_div & 0xff)
                + PcrI2sClkmConf::I2S_CLKM_SEL.val(0) // XTAL
                + PcrI2sClkmConf::I2S_CLKM_EN.val(1),
        );

        // RX clock: same, plus MCLK_SEL = 1 (use TX clock for MCLK).
        self.pcr.i2s_rx_clkm_conf.modify(
            PcrI2sClkmConf::I2S_CLKM_DIV_NUM.val(mclk_div & 0xff)
                + PcrI2sClkmConf::I2S_CLKM_SEL.val(0) // XTAL
                + PcrI2sClkmConf::I2S_CLKM_EN.val(1)
                + PcrI2sClkmConf::I2S_MCLK_SEL.val(1),
        );

        // Program BCK divider and bit width in CONF1.
        let bits_field = (bits as u32 - 1) & 0x1f;
        let hsb_field = (half_sample_bits - 1) & 0x3f;

        // TX_CONF1: BCK_DIV_NUM | BITS_MOD | HALF_SAMPLE_BITS | MSB_SHIFT (Philips).
        self.registers.tx_conf1.write(
            Conf1::BCK_DIV_NUM.val(bck_div_field)
                + Conf1::BITS_MOD.val(bits_field)
                + Conf1::HALF_SAMPLE_BITS.val(hsb_field)
                + Conf1::MSB_SHIFT.val(1),
        );

        // RX_CONF1: same layout.
        self.registers.rx_conf1.write(
            Conf1::BCK_DIV_NUM.val(bck_div_field)
                + Conf1::BITS_MOD.val(bits_field)
                + Conf1::HALF_SAMPLE_BITS.val(hsb_field)
                + Conf1::MSB_SHIFT.val(1),
        );

        Ok(())
    }

    /// Configure the TX (playback) path for the given format.
    fn configure_tx(&self, cfg: &I2sConfig) {
        // Reset TX.
        self.registers.tx_conf.write(TxConf::TX_RESET::SET + TxConf::TX_FIFO_RESET::SET);
        self.registers.tx_conf.set(0);

        // Build TX_CONF value.
        let mut conf = TxConf::TX_PCM_BYPASS::SET + TxConf::TX_STOP_EN::SET;
        if matches!(cfg.channel_mode, I2sChannelMode::Mono) {
            conf += TxConf::TX_MONO::SET + TxConf::TX_CHAN_EQUAL::SET;
        }
        // TDM and PDM are cleared (standard I2S only).
        self.registers.tx_conf.write(conf);

        // TDM_CTRL: enable channels 0 and 1, total = 2 channels for stereo.
        let (chan_en, tot_chan) = match cfg.channel_mode {
            I2sChannelMode::Stereo => (0x3, 2 - 1),
            I2sChannelMode::Mono => (0x1, 1 - 1),
        };
        self.registers.tx_tdm_ctrl.write(
            TdmCtrl::TDM_CHAN_EN.val(chan_en) + TdmCtrl::TDM_TOT_CHAN_NUM.val(tot_chan),
        );
    }

    /// Configure the RX (capture) path for the given format.
    fn configure_rx(&self, cfg: &I2sConfig) {
        // Reset RX.
        self.registers.rx_conf.write(RxConf::RX_RESET::SET + RxConf::RX_FIFO_RESET::SET);
        self.registers.rx_conf.set(0);

        let mut conf = RxConf::RX_PCM_BYPASS::SET;
        if matches!(cfg.channel_mode, I2sChannelMode::Mono) {
            conf += RxConf::RX_MONO::SET;
        }
        self.registers.rx_conf.write(conf);

        // RXEOF_NUM: number of RX data words before in_suc_eof fires.
        self.registers.rxeof_num.set(0x40);

        // TDM_CTRL: same as TX.
        let (chan_en, tot_chan) = match cfg.channel_mode {
            I2sChannelMode::Stereo => (0x3, 2 - 1),
            I2sChannelMode::Mono => (0x1, 1 - 1),
        };
        self.registers.rx_tdm_ctrl.write(
            TdmCtrl::TDM_CHAN_EN.val(chan_en) + TdmCtrl::TDM_TOT_CHAN_NUM.val(tot_chan),
        );
    }

    /// Apply TX register updates (clock-domain crossing).
    fn tx_update(&self) {
        self.registers.tx_conf.modify(TxConf::TX_UPDATE::SET);
        // TX_UPDATE is self-clearing; wait for it.
        for _ in 0..POLL_LIMIT {
            if self.registers.tx_conf.get() & TxConf::TX_UPDATE.mask == 0 {
                break;
            }
            core::hint::spin_loop();
        }
    }

    /// Apply RX register updates (clock-domain crossing).
    fn rx_update(&self) {
        self.registers.rx_conf.modify(RxConf::RX_UPDATE::SET);
        for _ in 0..POLL_LIMIT {
            if self.registers.rx_conf.get() & RxConf::RX_UPDATE.mask == 0 {
                break;
            }
            core::hint::spin_loop();
        }
    }
}

impl<const TX_CH: usize, const RX_CH: usize> PlatPeri for Esp32c6I2s0<TX_CH, RX_CH> {
    fn enable(&self) {
        // 1. Enable I2S APB clock + high-pulse reset.
        let conf_val = self.pcr.i2s_conf.get() | PcrI2sConf::I2S_CLK_EN.mask;
        self.pcr.i2s_conf.set(conf_val | PcrI2sConf::I2S_RST_EN.mask);
        self.pcr.i2s_conf.set(conf_val & !PcrI2sConf::I2S_RST_EN.mask);

        // 2. Reset the DMA channels and bind them to I2S0.
        Esp32c6GdmaChannel::<TX_CH>::reset_tx();
        Esp32c6GdmaChannel::<TX_CH>::set_tx_peri(PERI_I2S0);
        Esp32c6GdmaChannel::<RX_CH>::reset_rx();
        Esp32c6GdmaChannel::<RX_CH>::set_rx_peri(PERI_I2S0);
    }
    fn disable(&self) {
        // Stop TX/RX.
        self.registers.tx_conf.modify(TxConf::TX_START::CLEAR);
        self.registers.rx_conf.modify(RxConf::RX_START::CLEAR);
        // Gate the function clocks.
        self.pcr.i2s_tx_clkm_conf.modify(PcrI2sClkmConf::I2S_CLKM_EN::CLEAR);
        self.pcr.i2s_rx_clkm_conf.modify(PcrI2sClkmConf::I2S_CLKM_EN::CLEAR);
    }
}

impl<const TX_CH: usize, const RX_CH: usize> Configuration<I2sConfig>
    for Esp32c6I2s0<TX_CH, RX_CH>
{
    type Target = ();

    fn configure(&self, cfg: &I2sConfig) -> blueos_hal::err::Result<Self::Target> {
        let channels: u8 = match cfg.channel_mode {
            I2sChannelMode::Stereo => 2,
            I2sChannelMode::Mono => 1,
        };

        // Clock tree first.
        self.configure_clock(cfg.sample_rate, cfg.bits_per_sample, channels)?;

        // TX/RX format.
        self.configure_tx(cfg);
        self.configure_rx(cfg);

        // Push register updates across the clock domain.
        self.tx_update();
        self.rx_update();

        // Clear any pending I2S interrupts and disable them (we poll).
        self.registers.int_clr.write(I2sInt::TX_DONE.val(1) + I2sInt::RX_DONE.val(1));
        self.registers.int_ena.set(0);

        // FIFO hung timeout — keep the default (0x0810) to avoid stalls.
        self.registers.lc_hung_conf.set(0x0810);

        Ok(())
    }
}

impl<const TX_CH: usize, const RX_CH: usize> I2s<I2sConfig, ()>
    for Esp32c6I2s0<TX_CH, RX_CH>
{
    fn write(&self, buf: &[u8]) -> blueos_hal::err::Result<()> {
        if buf.is_empty() {
            return Ok(());
        }

        // Prepare the TX descriptor: single descriptor, suc_eof = true.
        let desc = unsafe { &mut *self.tx_desc.get() };
        *desc = DmaDescriptor::for_tx(buf.as_ptr() as *mut u8, buf.len(), true);

        // Start DMA and I2S TX.
        Esp32c6GdmaChannel::<TX_CH>::start_tx(desc);
        self.registers.tx_conf.modify(TxConf::TX_START::SET);

        // Wait for the DMA to finish.
        let result = Esp32c6GdmaChannel::<TX_CH>::wait_tx_done();

        // Stop TX so the next write can restart cleanly.
        self.registers.tx_conf.modify(TxConf::TX_START::CLEAR);

        result
    }

    fn read(&self, buf: &mut [u8]) -> blueos_hal::err::Result<()> {
        if buf.is_empty() {
            return Ok(());
        }

        let desc = unsafe { &mut *self.rx_desc.get() };
        *desc = DmaDescriptor::for_rx(buf.as_mut_ptr(), buf.len());

        Esp32c6GdmaChannel::<RX_CH>::start_rx(desc);
        self.registers.rx_conf.modify(RxConf::RX_START::SET);

        let result = Esp32c6GdmaChannel::<RX_CH>::wait_rx_done();

        self.registers.rx_conf.modify(RxConf::RX_START::CLEAR);

        result
    }
}
