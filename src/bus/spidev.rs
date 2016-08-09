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

use pcd::reg::Reg;

impl MFRC522Bus for Spidev {
	#[inline]
	fn register_read(&mut self, reg: Reg) -> bus::Result<u8> {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Read);
		let mut rx_buf: [u8; 1] = [0];
		try_map_err!(self.write(&[reg_addr]), ());
		try_map_err!(self.read(&mut rx_buf), ());
		trace!("{:?}/{:02x} -> {:?}", reg, reg_addr, rx_buf);

		Ok(rx_buf[0])
	}

	#[inline]
	fn register_write(&mut self, reg: Reg, value: u8) -> bus::Result<()> {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Write);
		trace!("{:?}/{:02x} <- {:02x}", reg, reg_addr, value);
		try_map_err!(self.write(&[reg_addr, value]), ());

		Ok(())
	}

	fn register_write_slice(&mut self, reg: Reg, values: &[u8]) -> bus::MultiResult<usize> {
		let reg_addr = bus::spi_reg_addr(reg, bus::Mode::Write);
		let mut tx_buf = Vec::with_capacity(values.len() + 1);
		tx_buf.push(reg_addr);
		tx_buf.extend_from_slice(values);
		let nwrit = try_map_err!(self.write(tx_buf.as_slice()), 0usize);
		trace!("{:?}/{:02x} <- |{}/{}| {:?}", reg, reg_addr, nwrit, tx_buf.len(), &tx_buf.as_slice()[1..]);

		Ok(nwrit)
	}
}
