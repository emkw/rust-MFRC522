/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! MFRC522Bus implementation for Spidev.

use std::io::Read;
use std::io::Write;

use spidev::Spidev;

use bus;
use bus::MFRC522Bus;

use pcd;

impl MFRC522Bus for Spidev {
	fn register_read(&mut self, reg: pcd::Reg) -> u8 {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Read);
		let mut rx_buf: [u8; 1] = [0];
		self.write(&[reg_addr]).unwrap();
		self.read(&mut rx_buf).unwrap();

		rx_buf[0]
	}

	fn register_write(&mut self, reg: pcd::Reg, value: u8) {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Write);
		self.write(&[reg_addr, value]).unwrap();
	}
}
