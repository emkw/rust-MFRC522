/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! This is not tested at all. Would appricciate testing & bug feedback.
//!
//! WARNING: It conflicts with Spidev implementation so only one of
//! `spidev` `i2cdev` features may be selected.

use i2cdev::core::I2CDevice;

use bus;
use bus::MFRC522Bus;

use pcd::reg::Reg;

impl<B: I2CDevice> MFRC522Bus for B {
	#[inline]
	fn register_read(&mut self, reg: Reg) -> bus::Result<u8> {
		let value = try_map_err!(self.smbus_read_byte_data(reg as u8), ());

		Ok(value)
	}

	#[inline]
	fn register_write(&mut self, reg: Reg, value: u8) -> bus::Result<()> {
		try_map_err!(self.smbus_write_byte_data(reg as u8, value), ());

		Ok(())
	}
}
