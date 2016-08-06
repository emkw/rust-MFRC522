/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! This is not tested at all. Would appricciate testing & bug feedback.

use i2cdev::core::I2CDevice;

use bus::MFRC522Bus;
use pcd;

impl<B> MFRC522Bus for B where B: I2CDevice
{


	fn register_read(&mut self, reg: pcd::Reg) -> u8 {
		self.smbus_read_byte_data(reg as u8).unwrap()
	}

	fn register_write(&mut self, reg: pcd::Reg, value: u8) {
		self.smbus_write_byte_data(reg as u8, value).unwrap()
	}
}
