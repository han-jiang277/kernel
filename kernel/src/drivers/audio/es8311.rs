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

//! Simple ES8311 audio codec driver.
//!
//! The ES8311 is controlled over I2C (address 0x18) and exchanges audio
//! data with the I2S peripheral. This driver handles only the I2C control
//! path — register configuration and verification. Audio data flows through
//! the I2S driver (`/dev/i2s0`).
//!
//! The `init()` method applies the register sequence for 16 kHz / 16-bit /
//! I2S-Philips / MCLK = 256×fs. The `verify()` method reads back a few
//! registers to confirm the codec is present and responding on the I2C bus.

use blueos_driver::i2c::I2cConfig;
use embedded_hal::i2c::I2c as HalI2c;

use crate::devices::{
    bus::{Bus, BusWrapper},
    i2c_core::block_i2c::BlockI2c,
};

/// ES8311 I2C slave address (7-bit, shifted left by the controller).
const ES8311_I2C_ADDR: u8 = 0x18;

/// Minimum delay (spin iterations) for the codec to settle after reset.
const RESET_SETTLE_SPINS: usize = 100_000;

/// Register-value pair for the ES8311 init sequence.
struct RegVal(u8, u8);

/// ES8311 codec driver — wraps the shared I2C bus.
pub struct Es8311Driver<T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static> {
    bus: BusWrapper<BlockI2c<T>>,
}

impl<T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static> Es8311Driver<T> {
    /// Create a new ES8311 driver from a reference to the I2C bus.
    pub fn new(bus: &Bus<BlockI2c<T>>) -> Self {
        Self {
            bus: bus.intf.clone(),
        }
    }

    /// Write a value to an ES8311 register over I2C.
    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), crate::error::Error> {
        self.bus
            .transaction(ES8311_I2C_ADDR, &mut [embedded_hal::i2c::Operation::Write(&[reg, val])])
    }

    /// Read a value from an ES8311 register over I2C.
    fn read_reg(&mut self, reg: u8) -> Result<u8, crate::error::Error> {
        let mut buf = [0u8; 1];
        self.bus.transaction(
            ES8311_I2C_ADDR,
            &mut [
                embedded_hal::i2c::Operation::Write(&[reg]),
                embedded_hal::i2c::Operation::Read(&mut buf),
            ],
        )?;
        Ok(buf[0])
    }

    /// Apply the full ES8311 init sequence for 16 kHz / 16-bit / I2S-Philips.
    pub fn init(&mut self) -> Result<(), crate::error::Error> {
        // Reset & power on.
        self.write_reg(0x00, 0x1F)?; // reset
        for _ in 0..RESET_SETTLE_SPINS {
            core::hint::spin_loop();
        }
        self.write_reg(0x00, 0x00)?; // clear reset
        self.write_reg(0x00, 0x80)?; // power on, slave serial port

        // Clock source & enable (MCLK from MCLK pin, not inverted, all clocks on).
        self.write_reg(0x01, 0x3F)?;

        // Clock dividers for MCLK=4.096MHz (256×16kHz), fs=16kHz.
        self.write_reg(0x02, 0x00)?; // pre_div=1, pre_multi=1x
        self.write_reg(0x03, 0x10)?; // single speed, adc_osr=0x10
        self.write_reg(0x04, 0x10)?; // dac_osr=0x10
        self.write_reg(0x05, 0x00)?; // adc_div=1, dac_div=1
        self.write_reg(0x06, 0x03)?; // bclk_div=4
        self.write_reg(0x07, 0x00)?; // lrck_h=0
        self.write_reg(0x08, 0xFF)?; // lrck_l=0xFF

        // I2S format (16-bit, Philips).
        self.write_reg(0x09, 0x0C)?; // DAC SDP: I2S, 16-bit
        self.write_reg(0x0A, 0x0C)?; // ADC SDP: I2S, 16-bit

        // Power up analog & DAC.
        self.write_reg(0x0D, 0x01)?; // power up analog
        self.write_reg(0x0E, 0x02)?; // enable analog PGA + ADC modulator
        self.write_reg(0x12, 0x00)?; // power-up DAC
        self.write_reg(0x13, 0x10)?; // enable HP output driver
        self.write_reg(0x1C, 0x6A)?; // ADC EQ bypass, DC offset cancel
        self.write_reg(0x37, 0x08)?; // DAC EQ bypass, fade off

        // Unmute & set volume.
        self.write_reg(0x31, 0x00)?; // unmute DAC
        self.write_reg(0x32, 0xFF)?; // volume = max

        Ok(())
    }

    /// Verify the codec is present by reading back a few registers and
    /// checking they match the values written during `init()`.
    ///
    /// Returns `Ok(())` if all checks pass, `Err(EIO)` otherwise.
    pub fn verify(&mut self) -> Result<(), crate::error::Error> {
        // Registers written during init that should retain their values.
        let checks: [RegVal; 4] = [
            RegVal(0x09, 0x0C), // DAC SDP: I2S, 16-bit
            RegVal(0x0A, 0x0C), // ADC SDP: I2S, 16-bit
            RegVal(0x31, 0x00), // unmute DAC
            RegVal(0x32, 0xFF), // volume = max
        ];

        for RegVal(reg, expected) in checks.iter() {
            match self.read_reg(*reg) {
                Ok(actual) => {
                    if actual != *expected {
                        log::warn!(
                            "[ES8311] verify failed: reg 0x{:02X} = 0x{:02X}, expected 0x{:02X}",
                            reg,
                            actual,
                            expected
                        );
                        return Err(crate::error::code::EIO);
                    }
                    log::info!(
                        "[ES8311] reg 0x{:02X} = 0x{:02X} (OK)",
                        reg,
                        actual
                    );
                }
                Err(e) => {
                    log::warn!("[ES8311] verify failed: cannot read reg 0x{:02X}: {:?}", reg, e);
                    return Err(e);
                }
            }
        }

        log::info!("[ES8311] codec verified: all register checks passed");
        Ok(())
    }
}
