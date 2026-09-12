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

mod config;
use crate::{
    arch::riscv::{local_irq_enabled, trap_entry, Context},
    kearly_println,
};
use blueos_driver::uart::esp32_usb_serial::Esp32UsbSerialIsr;
use blueos_hal::{isr::IsrDesc, Has8bitDataReg};

const LED_DEVICE_MAJOR: usize = 242;
const LED_B_DEVICE_MINOR: usize = 0;
const LED_R_DEVICE_MINOR: usize = 1;

pub type Spi2Impl =
    blueos_driver::spi::esp32c6_spi::Esp32c6Spi2<0x6008_1000, 0x6009_6000, 80_000_000>;

pub type ClockImpl =
    blueos_driver::systimer::esp32_sys_timer::Esp32SysTimer<0x6000_a000, 16_000_000>;

core::arch::global_asm!(
    "
.section .trap
.type _vector_table, @function

.option push
.balign 0x4
.option norelax
.option norvc

_vector_table:
    j {trap_entry}          // 0: Exception
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    j {trap_entry}
    ",
    trap_entry = sym trap_entry,
);

#[inline]
fn init_vector_table() {
    unsafe extern "C" {
        static _vector_table: u32;
    }
    let mut v = core::ptr::addr_of!(_vector_table) as usize;
    v |= 1; // set the least significant bit to enable vectored mode
    unsafe {
        core::arch::asm!(
            "csrw mtvec, {0}",
            in(reg) v,
            options(nostack, preserves_flags),
        );
    }
}

const PLIC_MX_BASE: usize = 0x2000_1000;
const PLIC_MX_ENABLE: usize = PLIC_MX_BASE;
const PLIC_MX_TYPE: usize = PLIC_MX_BASE + 0x4;
#[allow(dead_code)]
const PLIC_MX_CLEAR: usize = PLIC_MX_BASE + 0x8;
const PLIC_MX_EIP_STATUS: usize = PLIC_MX_BASE + 0xC;
const PLIC_MX_PRI: usize = PLIC_MX_BASE + 0x10;
const PLIC_MX_THRESH: usize = PLIC_MX_BASE + 0x90;

const MIDELEG_UEXT_BIT: usize = 1 << 8;
const MIDELEG_UTIMER_BIT: usize = 1 << 4;
const MIDELEG_USOFT_BIT: usize = 1 << 0;

const MIDELEG_DELEG_MASK: usize = MIDELEG_USOFT_BIT | MIDELEG_UTIMER_BIT | MIDELEG_UEXT_BIT;

const INTMTX_BASE: usize = 0x6001_0000;

const INTMTX_USB_SERIAL_JTAG_MAP: usize = INTMTX_BASE + 0xC0;

const INTMTX_SYSTIMER_TARGET0_MAP: usize = INTMTX_BASE + 0xE4;

const TARGET0_INT_NUM: usize = 16;

/* Watchdog timers enabled by the bootloader in flash-boot mode. Unlike C3
(whose RTC WDT lives in RTC_CNTL at 0x6000_8000), C6 splits its watchdogs:
the RTC/low-power watchdog moved to the LP_WDT block at 0x600B_1C00, while
0x6000_8000 is now Timer Group 0 (TIMG0), whose MWDT is *also* kept running
by the bootloader. If neither is disabled, the flash-boot watchdog fires a
few hundred ms after the app starts — the chip resets, the USB-Serial-JTAG
CDC port re-enumerates, and the host monitor (espflash) dies with a
`Broken pipe` read error. This is the C6 analogue of C3's RTC WDT-disable
block (see seeed_xiao_esp32c3/mod.rs). Addresses from esp-idf
soc/esp32c6/register/soc/reg_base.h + lp_wdt_reg.h.

LP_WDT layout: wdtconfig0 @ +0x00, wdtwprotect @ +0x18. wdt_en is bit 31,
wdt_flashboot_mod_en is bit 12. Both WDTs share the write-protect unlock
key 0x50D8_3AA1 (same as C3, confirmed in esp-hal rtc_cntl/timg drivers).

TIMG0 MWDT layout (standard across ESP32 chips, used by esp-hal timg.rs):
wdtconfig0 @ +0x48, wdtwprotect @ +0x64, wdt_en bit 31. */
const LP_WDT_BASE: usize = 0x600B_1C00;
const LP_WDT_CONFIG0: usize = LP_WDT_BASE; // wdtconfig0 @ +0x00
const LP_WDT_WPROTECT: usize = LP_WDT_BASE + 0x18;
const TIMG0_BASE: usize = 0x6000_8000;
const TIMG0_WDT_CONFIG0: usize = TIMG0_BASE + 0x48;
const TIMG0_WDT_WPROTECT: usize = TIMG0_BASE + 0x64;
const WDT_WKEY: u32 = 0x50D8_3AA1;

const WDT_EN_BIT: u32 = 1 << 31;

const WDT_FLASHBOOT_MOD_EN_BIT: u32 = 1 << 12; // bit 12

const USB_SERIAL_JTAG_INT_NUM: usize = 15;

// Access Path Manager (APM) filter registers.
const LP_APM_FUNC_CTRL: usize = 0x600B_3800 + 0xC4;
const LP_APM0_FUNC_CTRL: usize = 0x6009_9800 + 0xC4;
const HP_APM_FUNC_CTRL: usize = 0x6009_9000 + 0xC4;

#[allow(dead_code)]
const MIE_MEIE_BIT: usize = 1 << 11;

#[inline]
unsafe fn write32(addr: usize, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) };
}

#[inline]
unsafe fn read32(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

unsafe fn route_source(map_reg: usize, line: usize, prio: u32) {
    unsafe {
        let mut mie: usize;
        core::arch::asm!(
            "csrr {mie}, mie",
            "csrw mie, zero",
            mie = out(reg) mie,
            options(nostack, preserves_flags),
        );
        write32(map_reg, line as u32);
        let t = read32(PLIC_MX_TYPE);
        write32(PLIC_MX_TYPE, t & !(1u32 << line));
        write32(PLIC_MX_PRI + line * 4, prio & 0xF);
        let en = read32(PLIC_MX_ENABLE);
        write32(PLIC_MX_ENABLE, en | (1u32 << line));
        mie |= 1usize << line;
        core::arch::asm!("fence io, io", options(nostack, preserves_flags));
        core::arch::asm!(
            "csrw mie, {mie}",
            mie = in(reg) mie,
            options(nostack, preserves_flags),
        );
    }
}

#[inline]
unsafe fn disable_wdt(wprotect: usize, config0: usize, flashboot_mask: u32) {
    unsafe {
        write32(wprotect, WDT_WKEY); // unlock
        let cfg = read32(config0);
        write32(config0, cfg & !(WDT_EN_BIT | flashboot_mask));
        write32(wprotect, 0); // re-lock
    }
}

// Ana I2C master register block base (DR_REG_I2C_ANA_MST_BASE)
const I2C_ANA_MST_BASE: usize = 0x600A_F800;
const I2C_ANA_MST_I2C0_CTRL: usize = I2C_ANA_MST_BASE; // +0x00
const I2C_ANA_MST_I2C1_CTRL: usize = I2C_ANA_MST_BASE + 0x04;
const I2C_MST_ANA_CONF0: usize = I2C_ANA_MST_BASE + 0x18; // BBPLL calibration control
const I2C_MST_ANA_CONF1: usize = I2C_ANA_MST_BASE + 0x1C; // slave RD mask
const I2C_MST_ANA_CONF2: usize = I2C_ANA_MST_BASE + 0x20; // slave MST_SEL

// MODEM_LPCON.clk_conf @ 0x600A_F018, bit2 = clk_i2c_mst_en (ana I2C master clock)
const MODEM_LPCON_CLK_CONF_FOR_I2C: usize = 0x600A_F018;
const CLK_I2C_MST_EN_BIT: u32 = 1 << 2;

// BBPLL calibration control bits in I2C_MST_ANA_CONF0
const BBPLL_STOP_FORCE_HIGH: u32 = 1 << 2; // bit2: set to stop calibration (stop=high)
const BBPLL_STOP_FORCE_LOW: u32 = 1 << 3; // bit3: set to start calibration (stop=low)
const BBPLL_CAL_DONE: u32 = 1 << 24; // bit24 (RO): 1 = calibration done

// I2C_CTRL field layout (I2C_ANA_MST_I2C0/1_CTRL)
const REGI2C_RTC_SLAVE_ID_S: u32 = 0;
const REGI2C_RTC_ADDR_S: u32 = 8;
const REGI2C_RTC_DATA_S: u32 = 16;
const REGI2C_RTC_WR_CNTL: u32 = 1 << 24; // bit24: 0=read, 1=write
const REGI2C_RTC_BUSY: u32 = 1 << 25; // bit25 (RO): 1=busy

// Ana I2C slave block ids (regi2c_defs.h / patches esp_rom_regi2c_esp32h2.c)
const REGI2C_BBPLL: u8 = 0x66;
const REGI2C_DIG_REG: u8 = 0x6D;

// Slave select masks for I2C_MST_ANA_CONF1 (RD_MASK: clear target bit, keep others)
// CONF1 bit6=BIAS / bit7=BBPLL / bit8=ULP / bit9=SAR / bit10=DIG_REG
const REGI2C_BBPLL_RD_MASK: u32 = !(1 << 7) & 0x00FF_FFFF;
const REGI2C_DIG_REG_RD_MASK: u32 = !(1 << 10) & 0x00FF_FFFF;
// Slave select bits for I2C_MST_ANA_CONF2 (MST_SEL: 1=route to I2C1)
const REGI2C_BBPLL_MST_SEL: u32 = 1 << 9;
const REGI2C_DIG_REG_MST_SEL: u32 = 1 << 12;

// ROM ets_delay_us absolute address = 0x40000040 (strong symbol, esp32c6.rom.ld:31).
// link.x does not INCLUDE esp32c6.rom.ld, so the symbol cannot be extern-imported
// (would be undefined reference); instead call via a raw-address function pointer,
// bypassing linker symbol resolution. Signature: void ets_delay_us(uint32_t us),
// RISC-V calling convention: a0 = us.
const ETS_DELAY_US: usize = 0x4000_0040;
#[inline]
unsafe fn ets_delay_us(us: u32) {
    let f: unsafe extern "C" fn(u32) = core::mem::transmute(ETS_DELAY_US);
    unsafe { f(us) };
}

pub(crate) fn handle_intc_irq(ctx: &Context, mcause: usize, mtval: usize) {
    let _ = (ctx, mtval);
    match mcause & 0xff {
        // WiFi interrupt: libnet80211 aggregates the WIFI_MAC/WIFI_PWR sources
        // into CPU intr 1 (see esp32_wlan::api::set_isr ISR_INTERRUPT_1); once the
        // trap fires it is dispatched here. Structurally identical to the C3 board
        // seeed_xiao_esp32c3/mod.rs:97-100.
        0 | 1 => {
            #[cfg(enable_net)]
            {
                crate::net::link::esp32_wlan::api::ISR_INTERRUPT_1.dispatch();
            }
        }
        TARGET0_INT_NUM => {
            ClockImpl::clear_interrupt();
            crate::time::handle_clock_interrupt();
        }
        USB_SERIAL_JTAG_INT_NUM => {
            ESP32_USB_SERIAL_ISR.service_isr();
        }
        _ => {}
    }
}

pub(crate) fn init() {
    assert!(!local_irq_enabled());

    crate::boot::init_runtime();
    crate::boot::init_heap();
    init_vector_table();

    blueos_driver::systimer::esp32_sys_timer::Esp32SysTimer::<0x6000_a000, 16_000_000>::init();

    unsafe {
        // Disable the three Access Path Manager (APM) filters early. Their func_ctrl
        // defaults to TEE-only, denying all REE-mode masters — including WiFi DMA.
        // Ported from esp-hal-1.1.1 src/soc/esp32c6/mod.rs:31-49 pre_init.
        // [ROOT CAUSE] Confirmed via 4-round bisect (2026-08-14): of the four
        // esp-hal-vs-blueos WiFi-interrupt gaps, disabling APM is the sole change
        // that makes WiFi interrupts fire. REE-mode WiFi DMA is denied bus access
        // by the TEE-only func_ctrl default until APM is disabled; without it the
        // MAC can never complete a DMA fetch, so the "rx done" interrupt is never
        // raised regardless of INTMTX/PLIC_MX/mie configuration.
        write32(LP_APM_FUNC_CTRL, 0);
        write32(LP_APM0_FUNC_CTRL, 0);
        write32(HP_APM_FUNC_CTRL, 0);

        // PLIC_MX threshold: only interrupts with prio > thresh fire.
        // [BISECT-ROUND1] temporarily restored to 1 (original value) to test
        // whether threshold=0 was the WiFi-interrupt root cause. If scan still
        // finds APs with thresh=1, this change was NOT the root cause.
        write32(PLIC_MX_THRESH, 1);
        route_source(INTMTX_USB_SERIAL_JTAG_MAP, USB_SERIAL_JTAG_INT_NUM, 15);
        route_source(INTMTX_SYSTIMER_TARGET0_MAP, TARGET0_INT_NUM, 15);
    }

    // ------------------------------------------------------------------
    // System clock tree configure(): MSPI HS divider + SOC_ROOT_CLK selection.
    //
    // Ported from esp-hal-1.1.1 src/soc/esp32c6/clocks.rs::ClockConfig::configure()
    // (clocks.rs:102-117). esp-hal forces the MSPI source-clock HS divider to /6
    // (=80MHz) before switching to PLL, because C6's MSPI HS divider reset default
    // is 120MHz and is unusable before calibration — if not preset, flash
    // instruction/data access errors out under high load after the PLL switch.
    //   PLL = 480MHz, div_num=5 → 480/(5+1)=80MHz (esp-hal MspiFastHsClkDivisor::_5).
    // SOC_ROOT_CLK selects PLL (soc_clk_sel[1:0]=1), matching esp-hal soc_root_clk=Pll.
    //
    // Offsets from the local PAC esp32c6-0.23.0 (the #[doc] address comments on each
    // register accessor in pcr.rs — these are the svd2rust-generated authoritative
    // in-block offsets; do NOT count RegisterBlock field ordinals, since field
    // declaration order != hardware address order):
    //   PCR base = 0x6009_6000 (lib.rs:692)
    //   PCR_SYSCLK_CONF    @ +0x110 → 0x6009_6110 (pcr.rs:383 "0x110 - SYSCLK ...")
    //     soc_clk_sel = Bits[1:0] (0=XTAL, 1=SPLL, 2=FOSC). TRM 7.2.4.3: WiFi/BLE
    //     only works when soc_clk_sel=1 (PLL), so must explicitly switch to PLL.
    //   PCR_MSPI_CLK_CONF  @ +0x1c  → 0x6009_601C (pcr.rs:97 "0x1c - MSPI_CLK ...")
    //     mspi_fast_hs_div_num = Bits[7:0] (value 5 = div6 → 480MHz/6 = 80MHz,
    //     esp-hal MspiFastHsClkDivisor::_5).
    unsafe {
        const PCR_BASE: usize = 0x6009_6000;
        const PCR_SYSCLK_CONF: usize = PCR_BASE + 0x110;
        const PCR_MSPI_CLK_CONF: usize = PCR_BASE + 0x1c;

        // soc_clk_sel = Bits[16:17] (PCR_SOC_CLK_SEL_S=16, IDF pcr_reg.h:1621 +
        // PAC sysclk_conf.rs). 0=XTAL, 1=SPLL(PLL). WiFi/BLE only works under PLL,
        // so must switch to 1. Note the field is at bit16-17, not bit0-1 (bit0-7 is
        // LS_DIV_NUM). Preserve other bits.
        let v = read32(PCR_SYSCLK_CONF);
        write32(PCR_SYSCLK_CONF, (v & !(0x3 << 16)) | (0x1 << 16));

        // mspi_fast_hs_div_num = Bits[8:15] (PCR_MSPI_FAST_HS_DIV_NUM_S=8). Value 5
        // = div6 → 480MHz/6 = 80MHz. Note the field is at bit8-15, not bit0-7. Preserve other bits.
        let v = read32(PCR_MSPI_CLK_CONF);
        write32(PCR_MSPI_CLK_CONF, (v & !(0xFF << 8)) | (5 << 8));
    }

    // ------------------------------------------------------------------
    // System clock tree configure(): MSPI HS divider + SOC_ROOT_CLK select.
    //
    // Ported from esp-hal-1.1.1 src/soc/esp32c6/clocks.rs::ClockConfig::configure()
    // (clocks.rs:102-117). esp-hal forces the MSPI source clock HS divider to
    // /6 (=80MHz) before switching to PLL, because the C6 MSPI HS divider resets
    // to 120MHz and is unusable until calibrated — if not preset, flash
    // instruction/data access breaks under high load after switching to PLL.
    //   PLL = 480MHz, div_num=5 → 480/(5+1)=80MHz (esp-hal MspiFastHsClkDivisor::_5).
    // SOC_ROOT_CLK selects PLL (soc_clk_sel[1:0]=1), matching esp-hal soc_root_clk=Pll.
    //
    // Offsets are taken from the local PAC esp32c6-0.23.0 (the #[doc] address
    // comments on each register accessor in pcr.rs — these are the svd2rust-
    // generated authoritative intra-block offsets; do NOT count RegisterBlock
    // field indices, since field declaration order != hardware address order):
    //   PCR base = 0x6009_6000 (lib.rs:692)
    //   PCR_SYSCLK_CONF    @ +0x110 → 0x6009_6110 (pcr.rs:383 "0x110 - SYSCLK ...")
    //     soc_clk_sel = Bits[1:0] (0=XTAL, 1=SPLL, 2=FOSC). TRM 7.2.4.3: WiFi/BLE
    //     only work when soc_clk_sel=1 (PLL), so an explicit switch to PLL is required.
    //   PCR_MSPI_CLK_CONF  @ +0x1c  → 0x6009_601C (pcr.rs:97 "0x1c - MSPI_CLK ...")
    //     mspi_fast_hs_div_num = Bits[7:0] (value 5 = div6 → 480MHz/6 = 80MHz,
    //     esp-hal MspiFastHsClkDivisor::_5).
    // Note: the system boots from flash, so the bootloader has already started the
    // PLL and set MSPI to 80MHz; this block mainly aligns with esp-hal's standard
    // init and guards against edge cases — a "should-do but was missing" cleanup.
    unsafe {
        const PCR_BASE: usize = 0x6009_6000;
        const PCR_SYSCLK_CONF: usize = PCR_BASE + 0x110;
        const PCR_MSPI_CLK_CONF: usize = PCR_BASE + 0x1c;

        // soc_clk_sel = Bits[16:17] (PCR_SOC_CLK_SEL_S=16, IDF pcr_reg.h:1621 +
        // PAC sysclk_conf.rs). 0=XTAL, 1=SPLL(PLL). WiFi/BLE only work under PLL,
        // so must switch to 1. Note the field is at bit16-17, not bit0-1
        // (bit0-7 is LS_DIV_NUM). Preserve other bits.
        let v = read32(PCR_SYSCLK_CONF);
        write32(PCR_SYSCLK_CONF, (v & !(0x3 << 16)) | (0x1 << 16));

        // mspi_fast_hs_div_num = Bits[8:15] (PCR_MSPI_FAST_HS_DIV_NUM_S=8). Value 5
        // = div6 → 480MHz/6 = 80MHz. Note the field is at bit8-15, not bit0-7.
        // Preserve other bits.
        let v = read32(PCR_MSPI_CLK_CONF);
        write32(PCR_MSPI_CLK_CONF, (v & !(0xFF << 8)) | (5 << 8));
    }

    unsafe {
        disable_wdt(LP_WDT_WPROTECT, LP_WDT_CONFIG0, 1 << 12);
        disable_wdt(TIMG0_WDT_WPROTECT, TIMG0_WDT_CONFIG0, 0);
    }

    // ------------------------------------------------------------------
    // WiFi modem clock enable: this is the C6 counterpart of the C3 board
    // (seeed_xiao_esp32c3/mod.rs:149-172 power_domain.enable_wifi() + writing
    // SYSTEM_WIFI_CLK_EN_REG); C6 previously omitted it, which left the driver
    // control path working (scan start / ScanDone normal) but RF RX receiving no
    // 802.11 frames at all — recv_cb_sta never called, scan number=0.
    //
    // Ported from esp-radio 0.18 src/radio_clocks/clocks_ll/esp32c6.rs::enable_wifi(true).
    // Register base / field offsets from the local PAC esp32c6-0.23.0 (same as
    // esp-hal 1.1.1):
    //   MODEM_SYSCON @ 0x600A_9800, RegisterBlock first field test_conf @0x00,
    //     so clk_conf1 @ +0x14 → 0x600A_9814 (modem_rst_conf @ +0x10 → 0x600A_9810,
    //     consistent with the wifi_reset_mac note below, confirming the offset).
    //   MODEM_LPCON  @ 0x600A_F000, clk_conf is the 7th field @ +0x18 → 0x600A_F018.
    // Use RMW (read-modify-write) to preserve other bits, only setting the
    // wifi/fe domain clock-enable bits.
    unsafe {
        const MODEM_SYSCON_CLK_CONF1: usize = 0x600A_9814;
        // clk_conf1 wifi/fe clock-enable bits (16 bits total; PAC esp32c6/clk_conf1.rs reader bit positions):
        //   bit0-10: wifibb_22m/40m/44m/80m/40x/80x/40x1/80x1/160x1 + wifimac + wifi_apb
        //   bit13-16: fe_80m / fe_160m / fe_cal_160m / fe_apb
        // bit11 (fe_20m) and bit12 (fe_40m) are not in the enable_wifi set range, keep original value.
        // Mask = bit0-10 | bit13-16 = 0x07FF | 0x1E000 = 0x1E7FF (bit11/12 MUST stay 0).
        const CLK_CONF1_WIFI_FE_MASK: u32 = 0x0001_E7FF; // bit0-10 | bit13-16
        let v = read32(MODEM_SYSCON_CLK_CONF1);
        write32(MODEM_SYSCON_CLK_CONF1, v | CLK_CONF1_WIFI_FE_MASK);

        const MODEM_LPCON_CLK_CONF: usize = 0x600A_F018;
        // bit0 clk_wifipwr_en | bit1 clk_coex_en (PAC esp32c6/modem_lpcon/clk_conf.rs)
        const LPCON_WIFIPWR_COEX_MASK: u32 = 0x3;
        let v = read32(MODEM_LPCON_CLK_CONF);
        write32(MODEM_LPCON_CLK_CONF, v | LPCON_WIFIPWR_COEX_MASK);

        // ------------------------------------------------------------------
        // PMU ICG gating + power_st state mapping + wifi_lp_clk_conf:
        // Ported from esp-radio 0.18 src/radio_clocks/clocks_ll/esp32c6.rs::init_clocks().
        //
        // Register offsets from the #[doc] address comments on each RegisterBlock
        // accessor in PAC esp32c6-0.23.0.
        const PMU_BASE: usize = 0x600b_0000;
        const PMU_HP_SLEEP_ICG_MODEM: usize = PMU_BASE + 0x74;
        const PMU_HP_MODEM_ICG_MODEM: usize = PMU_BASE + 0x40;
        const PMU_HP_ACTIVE_ICG_MODEM: usize = PMU_BASE + 0x0C;
        const ICG_MODEM_CODE_FIELD: u32 = 0b11 << 30; // bits[31:30]
                                                      // sleep code = 0: just clear this field
        let v = read32(PMU_HP_SLEEP_ICG_MODEM);
        write32(PMU_HP_SLEEP_ICG_MODEM, v & !ICG_MODEM_CODE_FIELD);
        // modem code = 1
        let v = read32(PMU_HP_MODEM_ICG_MODEM);
        write32(
            PMU_HP_MODEM_ICG_MODEM,
            (v & !ICG_MODEM_CODE_FIELD) | (1 << 30),
        );
        // active code = 2
        let v = read32(PMU_HP_ACTIVE_ICG_MODEM);
        write32(
            PMU_HP_ACTIVE_ICG_MODEM,
            (v & !ICG_MODEM_CODE_FIELD) | (2 << 30),
        );

        const PMU_IMM_MODEM_ICG: usize = PMU_BASE + 0xDC;
        write32(PMU_IMM_MODEM_ICG, 1 << 31);

        const PMU_IMM_SLEEP_SYSCLK: usize = PMU_BASE + 0xD0;
        write32(PMU_IMM_SLEEP_SYSCLK, 1 << 28);

        const MODEM_SYSCON_CLK_CONF_POWER_ST: usize = 0x600A_980C;

        const SYSCON_POWER_ST_HI: u32 =
            (6 << 28) | (4 << 24) | (6 << 20) | (6 << 16) | (6 << 12) | (6 << 8);
        let lo = read32(MODEM_SYSCON_CLK_CONF_POWER_ST) & 0xFF;
        write32(MODEM_SYSCON_CLK_CONF_POWER_ST, SYSCON_POWER_ST_HI | lo);

        const MODEM_LPCON_CLK_CONF_POWER_ST: usize = 0x600A_F020;
        const LPCON_POWER_ST_HI: u32 = (6 << 28) | (6 << 24) | (6 << 20) | (6 << 16);
        let lo = read32(MODEM_LPCON_CLK_CONF_POWER_ST) & 0xFFFF;
        write32(MODEM_LPCON_CLK_CONF_POWER_ST, LPCON_POWER_ST_HI | lo);

        const MODEM_LPCON_WIFI_LP_CLK_CONF: usize = 0x600A_F00C;
        const LPCON_LP_CLK_SEL_MASK: u32 = 0b1111; // bit0-3 four sel bits
        const LPCON_LP_DIV_NUM_MASK: u32 = 0xFFF0; // bits[15:4] div_num
        let v = read32(MODEM_LPCON_WIFI_LP_CLK_CONF);
        // Clear div_num then set the 4 sel bits, preserving the rest
        write32(
            MODEM_LPCON_WIFI_LP_CLK_CONF,
            (v & !LPCON_LP_DIV_NUM_MASK) | LPCON_LP_CLK_SEL_MASK,
        );
    }
}

crate::define_peripheral! {
    (console_uart, blueos_driver::uart::esp32_usb_serial::Esp32UsbSerial<0x6000_F000>,
     blueos_driver::uart::esp32_usb_serial::Esp32UsbSerial::<0x6000_F000>::new()),
    (spi2, Spi2Impl, Spi2Impl::new()),
    (i2c0, blueos_driver::i2c::esp32_i2c::Esp32I2c,
     blueos_driver::i2c::esp32_i2c::Esp32I2c::new_c6(
         0x6000_4000,
         0x6009_6000,
         40_000_000,
     )),
    (lcd_cs, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(
         blueos_kconfig::CONFIG_CO5300_CS_GPIO as u8)),
    (max7219_cs, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(
         blueos_kconfig::CONFIG_MAX7219_CS_GPIO as u8)),
    (touch_rst, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(
         blueos_kconfig::CONFIG_CST9220_RST_GPIO as u8)),
    (st7796_cs, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(
         blueos_kconfig::CONFIG_ST7796_CS_GPIO as u8)),
    (st7796_dc, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(
         blueos_kconfig::CONFIG_ST7796_DC_GPIO as u8)),
    (st7796_rst, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(
         blueos_kconfig::CONFIG_ST7796_RST_GPIO as u8)),
    (led_b, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(0)),
    (led_r, blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
     blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin::new(1)),
}

crate::define_bus! {
    (spi2_bus, crate::devices::spi_core::block_spi::BlockSpi<
        Spi2Impl,
        blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
    >,
        #[cfg(co5300)]
        (co5300, crate::drivers::lcd::co5300::Co5300Config<
            blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
            Co5300PanelSpec,
        >,
            crate::drivers::lcd::co5300::Co5300Config::new(
                get_device!(lcd_cs),
            )
        ),
        #[cfg(st7796)]
        (st7796, crate::drivers::lcd::st7796::St7796Config<
            blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
        >,
            crate::drivers::lcd::st7796::St7796Config::<
                blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
            > {
                rst: get_device!(st7796_rst),
                dc: get_device!(st7796_dc),
                cs: Some(get_device!(st7796_cs)),
                orientation: mipidsi::options::Orientation::new()
                    .rotate(mipidsi::options::Rotation::Deg0),

            }
        ),
        #[cfg(max7219)]
        (max7219, crate::drivers::display::max7219::Max7219Config<
            blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
        >,
            crate::drivers::display::max7219::Max7219Config::<
                blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
            >::new(
                get_device!(max7219_cs),
                1,
                1,
            )
        ),
    ),
    (i2c0_bus, crate::devices::i2c_core::block_i2c::BlockI2c<
        blueos_driver::i2c::esp32_i2c::Esp32I2c,
    >,
        #[cfg(cst9220)]
        (cst9220, crate::drivers::input::cst9220::Cst9220Config<
            blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
        >,
            crate::drivers::input::cst9220::Cst9220Config {
                rst: get_device!(touch_rst),
            }
        ),
        #[cfg(bme280)]
        (bme280, crate::drivers::sensor::bme280::Bme280Config,
            crate::drivers::sensor::bme280::Bme280Config::new(0x76)
        ),
    ),
}

#[cfg(any(co5300, cst9220, st7796, gpio, bme280, max7219))]
crate::define_pin_states!(
    blueos_driver::pinctrl::esp32c6_pinctrl::Esp32c6IoMuxPinctrl,
    #[cfg(co5300)]
    (
        blueos_kconfig::CONFIG_CO5300_SCLK_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        Some(63),
        None,
        false,
        false
    ),
    #[cfg(co5300)]
    (
        blueos_kconfig::CONFIG_CO5300_SIO0_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        Some(65),
        None,
        false,
        false
    ),
    #[cfg(co5300)]
    (
        blueos_kconfig::CONFIG_CO5300_SIO1_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        Some(64),
        None,
        false,
        false
    ),
    #[cfg(co5300)]
    (
        blueos_kconfig::CONFIG_CO5300_SIO2_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        Some(67),
        None,
        false,
        false
    ),
    #[cfg(co5300)]
    (
        blueos_kconfig::CONFIG_CO5300_SIO3_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        Some(66),
        None,
        false,
        false
    ),
    #[cfg(co5300)]
    (
        blueos_kconfig::CONFIG_CO5300_CS_GPIO as u8,
        1,
        false,
        true,
        false,
        2,
        Some(128),
        None,
        true,
        false
    ),
    // CST9220 uses the board's ESP32_SDA line.
    #[cfg(cst9220)]
    (
        blueos_kconfig::CONFIG_CST9220_SDA_GPIO as u8,
        1,
        true,
        true,
        false,
        2,
        Some(46),
        Some(46),
        false,
        true
    ),
    // CST9220 uses the board's ESP32_SCL line.
    #[cfg(cst9220)]
    (
        blueos_kconfig::CONFIG_CST9220_SCL_GPIO as u8,
        1,
        true,
        true,
        false,
        2,
        Some(45),
        Some(45),
        false,
        true
    ),
    #[cfg(cst9220)]
    (
        blueos_kconfig::CONFIG_CST9220_INT_GPIO as u8,
        1,
        true,
        true,
        false,
        2,
        None,
        None,
        false,
        false
    ),
    #[cfg(cst9220)]
    (
        blueos_kconfig::CONFIG_CST9220_RST_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        None,
        None,
        true,
        false
    ),
    // ST7796 SPI clock routed through the GPIO matrix (FSPICLK_OUT).
    #[cfg(st7796)]
    (
        blueos_kconfig::CONFIG_ST7796_SCLK_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        Some(63),
        None,
        false,
        false
    ),
    // ST7796 SPI MOSI routed through the GPIO matrix (FSPID_OUT).
    #[cfg(st7796)]
    (
        blueos_kconfig::CONFIG_ST7796_MOSI_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        Some(65),
        None,
        false,
        false
    ),
    // ST7796 chip select driven by software GPIO output.
    #[cfg(st7796)]
    (
        blueos_kconfig::CONFIG_ST7796_CS_GPIO as u8,
        1,
        false,
        true,
        false,
        2,
        None,
        None,
        true,
        false
    ),
    // ST7796 data/command line driven by software GPIO output.
    #[cfg(st7796)]
    (
        blueos_kconfig::CONFIG_ST7796_DC_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        None,
        None,
        true,
        false
    ),
    // ST7796 reset line driven by software GPIO output.
    #[cfg(st7796)]
    (
        blueos_kconfig::CONFIG_ST7796_RST_GPIO as u8,
        1,
        false,
        false,
        false,
        2,
        None,
        None,
        true,
        false
    ),
    // BME280 uses the board's ESP32_SDA line (I2CEXT0_SDA).
    #[cfg(bme280)]
    (
        blueos_kconfig::CONFIG_BME280_SDA_GPIO as u8,
        1,
        true,
        true,
        false,
        2,
        Some(46),
        Some(46),
        false,
        true
    ),
    // BME280 uses the board's ESP32_SCL line (I2CEXT0_SCL).
    #[cfg(bme280)]
    (
        blueos_kconfig::CONFIG_BME280_SCL_GPIO as u8,
        1,
        true,
        true,
        false,
        2,
        Some(45),
        Some(45),
        false,
        true
    ),
    // MAX7219 chip select driven by software GPIO output.
    #[cfg(max7219)]
    (
        blueos_kconfig::CONFIG_MAX7219_CS_GPIO as u8,
        1,
        false,
        true,
        false,
        2,
        None,
        None,
        true,
        false
    ),
    #[cfg(gpio)]
    (0, 1, false, true, false, 2, None, None, true, false), // led blue
    #[cfg(gpio)]
    (1, 1, false, true, false, 2, None, None, true, false), // led red
);

#[cfg(not(any(co5300, cst9220, st7796, gpio, bme280, max7219)))]
crate::define_pin_states!(None);

#[cfg(spi_core)]
type Spi2Bus = crate::devices::bus::Bus<
    crate::devices::spi_core::block_spi::BlockSpi<
        Spi2Impl,
        blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
    >,
>;

#[cfg(spi_core)]
static SPI2_BUS: spin::Once<alloc::sync::Arc<Spi2Bus>> = spin::Once::new();

#[cfg(spi_core)]
fn init_spi2_bus() -> crate::drivers::Result<&'static alloc::sync::Arc<Spi2Bus>> {
    use crate::devices::{bus::Bus, spi_core::block_spi::BlockSpi};
    use blueos_driver::spi::SpiConfig;

    if let Some(bus) = SPI2_BUS.get() {
        return Ok(bus);
    }

    let spi2 = get_device!(spi2);
    let mut spi_config = SpiConfig::qspi_display_default();
    #[cfg(max7219)]
    {
        // MAX7219 supports SPI mode 0 at up to 10 MHz, so cap the bus to
        // the lowest maximum frequency required by an attached device.
        spi_config.baudrate = 4_000_000;
    }
    let block = BlockSpi::new(spi2, get_device!(lcd_cs), &spi_config)
        .map_err(|_| crate::error::code::EIO)?;
    SPI2_BUS.call_once(|| alloc::sync::Arc::new(Bus::new(block)));
    SPI2_BUS.get().ok_or(crate::error::code::EIO)
}

#[cfg(spi_core)]
pub(crate) fn init_spi_bus() {
    use crate::drivers::InitDriver;

    let bus = init_spi2_bus().expect("failed to initialize ESP32-C6 SPI2");
    for device in crate::boards::get_bus_devices!(spi2_bus) {
        bus.register_device(device)
            .expect("failed to register ESP32-C6 SPI2 device");
    }

    #[cfg(co5300)]
    if let Ok(driver) = bus.probe_driver(&crate::drivers::lcd::co5300::Co5300DriverModule::<
        blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
        Co5300PanelSpec,
    >::new())
    {
        if let Err(error) = driver.init(bus) {
            kearly_println!("Failed to initialize CO5300 driver: {}", error);
            log::warn!("Failed to initialize CO5300 driver: {}", error);
        } else {
            kearly_println!("CO5300 framebuffer registered");
        }
    }

    #[cfg(st7796)]
    if let Ok(driver) = bus.probe_driver(&crate::drivers::lcd::st7796::St7796DriverModule::<
        blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
    >::new())
    {
        if let Err(error) = driver.init(bus) {
            kearly_println!("Failed to initialize ST7796 driver: {}", error);
            log::warn!("Failed to initialize ST7796 driver: {}", error);
        } else {
            kearly_println!("ST7796 framebuffer registered");
        }
    }

    #[cfg(max7219)]
    if let Ok(driver) = bus.probe_driver(&crate::drivers::display::max7219::Max7219DriverModule::<
        blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
    >::new())
    {
        if let Err(error) = driver.init(bus) {
            kearly_println!("Failed to initialize MAX7219 driver: {}", error);
            log::warn!("Failed to initialize MAX7219 driver: {}", error);
        } else {
            kearly_println!("MAX7219 LED matrix registered as /dev/max7219");
        }
    } else {
        kearly_println!("MAX7219 device description was not found on SPI2");
        log::warn!("MAX7219 device description was not found on SPI2");
    }
}

#[cfg(i2c_core)]
type I2c0Bus = crate::devices::bus::Bus<
    crate::devices::i2c_core::block_i2c::BlockI2c<blueos_driver::i2c::esp32_i2c::Esp32I2c>,
>;

#[cfg(i2c_core)]
static I2C0_BUS: spin::Once<alloc::sync::Arc<I2c0Bus>> = spin::Once::new();

#[cfg(i2c_core)]
fn init_i2c0_bus() -> crate::drivers::Result<&'static alloc::sync::Arc<I2c0Bus>> {
    use crate::devices::{bus::Bus, i2c_core::block_i2c::BlockI2c};

    if let Some(bus) = I2C0_BUS.get() {
        return Ok(bus);
    }

    let block = BlockI2c::new(get_device!(i2c0)).map_err(|_| crate::error::code::EIO)?;
    I2C0_BUS.call_once(|| alloc::sync::Arc::new(Bus::new(block)));
    I2C0_BUS.get().ok_or(crate::error::code::EIO)
}

pub(crate) fn init_i2c_bus() {
    #[cfg(any(cst9220, bme280))]
    {
        use crate::drivers::InitDriver;

        let bus = init_i2c0_bus().expect("failed to initialize ESP32-C6 I2C0");
        for device in crate::boards::get_bus_devices!(i2c0_bus) {
            bus.register_device(device)
                .expect("failed to register ESP32-C6 I2C0 device");
        }

        #[cfg(cst9220)]
        if let Ok(driver) =
            bus.probe_driver(&crate::drivers::input::cst9220::Cst9220DriverModule::<
                blueos_driver::gpio::esp32c6_gpio::Esp32c6GpioOutputPin,
            >::new())
        {
            if let Err(error) = driver.init(bus) {
                kearly_println!("Failed to initialize CST9220 driver: {}", error);
                log::warn!("Failed to initialize CST9220 driver: {}", error);
            } else {
                kearly_println!("CST9220 touch device registered as /dev/cst9220");
            }
        } else {
            kearly_println!("CST9220 device description was not found on I2C0");
            log::warn!("CST9220 device description was not found on I2C0");
        }

        #[cfg(bme280)]
        if let Ok(driver) = bus.probe_driver(&crate::drivers::sensor::bme280::Bme280DriverModule) {
            if let Err(error) = driver.init(bus) {
                kearly_println!("Failed to initialize BME280 driver: {}", error);
                log::warn!("Failed to initialize BME280 driver: {}", error);
            } else {
                kearly_println!("BME280 sensor registered as /dev/bme2800");
            }
        } else {
            kearly_println!("BME280 device description was not found on I2C0");
            log::warn!("BME280 device description was not found on I2C0");
        }
    }
}
pub(crate) fn init_gpio() {
    crate::devices::gpio::GeneralGpio::new(
        get_device!(led_b),
        Some(crate::devices::gpio::Level::High),
    )
    .register(
        alloc::string::String::from("led_b"),
        crate::devices::DeviceId::new(LED_DEVICE_MAJOR, LED_B_DEVICE_MINOR),
    )
    .expect("Failed to register led_b");
    crate::devices::gpio::GeneralGpio::new(
        get_device!(led_r),
        Some(crate::devices::gpio::Level::High),
    )
    .register(
        alloc::string::String::from("led_r"),
        crate::devices::DeviceId::new(LED_DEVICE_MAJOR, LED_R_DEVICE_MINOR),
    )
    .expect("Failed to register led_r");
}

#[inline(always)]
pub(crate) fn send_ipi(_hart: usize) {}

#[inline(always)]
pub(crate) fn clear_ipi(_hart: usize) {}

static ESP32_USB_SERIAL_ISR: Esp32UsbSerialIsr<0x6000_F000, crate::drivers::serial::Serial> =
    Esp32UsbSerialIsr::<0x6000_F000, _> {
        data: &crate::drivers::serial::TTY_SERIAL,
        tx_isr: Some(crate::drivers::serial::Serial::xmitchars),
        rx_isr: Some(crate::drivers::serial::Serial::recvchars),
    };
