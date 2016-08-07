/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! MFRC522Bus implementation for Spidev.
//!
//! WARNING: It conflicts with I2CDevice implementation so only one of
//! `spidev` `i2cdev` features may be selected.

use std::io::Read;
use std::io::Write;

use spidev::Spidev;

use bus;
use bus::MFRC522Bus;

use pcd;

impl MFRC522Bus for Spidev {
	#[inline]
	fn register_read(&mut self, reg: pcd::Reg) -> u8 {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Read);
		let mut rx_buf: [u8; 1] = [0];
		self.write(&[reg_addr]).unwrap();
		self.read(&mut rx_buf).unwrap();
		trace!("{:?}/{:02x} -> {:?}", reg, reg_addr, rx_buf);

		rx_buf[0]
	}

	#[inline]
	fn register_write(&mut self, reg: pcd::Reg, value: u8) {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Write);
		trace!("{:?}/{:02x} <- {:02x}", reg, reg_addr, value);
		self.write(&[reg_addr, value]).unwrap();
	}

	fn register_write_slice(&mut self, reg: pcd::Reg, values: &[u8]) {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Write);
		let mut tx_buf = Vec::with_capacity(values.len() + 1);
		tx_buf.push(reg_addr);
		tx_buf.extend_from_slice(values);
		let n = self.write(tx_buf.as_slice()).unwrap();
		trace!("{:?}/{:02x} <- |{}/{}| {:?}", reg, reg_addr, n, tx_buf.len(), &tx_buf.as_slice()[1..]);
	}
}
