/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! Trait for implementing communication with MFRC522 chip.
//! This is used to implement SPI/I2C/UART communication.
//!
//! When using SPI all addresses must be shifted one bit left in the "SPI address byte" (section 8.1.2.3)
//! This must be done in the trait implementation.

#[cfg(feature = "i2cdev")]
pub mod i2cdev;
#[cfg(feature = "spidev")]
pub mod spidev;

use pcd;


pub trait MFRC522Bus {
	/// Read single byte from MFRC522 register.
	fn register_read(&mut self, reg: pcd::Reg) -> u8;
	/// Write single byte to MFRC522 register.
	fn register_write(&mut self, reg: pcd::Reg, value: u8);

	#[inline]
	/// Write multiple bytes to a single MFRC522 register.
	fn register_write_slice(&mut self, reg: pcd::Reg, values: &[u8]) {
		for byte in values {
			self.register_write(reg, *byte);
		}
	}

	#[inline]
	/// Read multiple bytes from a single MFRC522 register.
	fn register_read_to_slice(&mut self, reg: pcd::Reg, buf: &mut [u8]) {
		for b in buf {
			*b = self.register_read(reg);
		}
	}

	/// Read multiple bytes from a single MFRC522 register,
	/// only partialy overwriting the first buffer byte.
	fn register_read_to_slice_align(&mut self, reg: pcd::Reg, buf: &mut [u8], rx_align: u8) {
		if buf.len() > 0 {
			let buf_tail = if rx_align != 0 {
				let (buf_head, buf_tail) = buf.split_at_mut(1);
				// Only update bit positions rxAlign..7 in buf[0]
				// Create bit mask for bit positions rxAlign..7
				let mask = (!0_u8).checked_shl(rx_align as u32).unwrap_or(0);
				let value = self.register_read(reg);
				// Apply mask to both current value of buf[0] and the new data in value.
				buf_head[0] = (buf_head[0] & !mask) | (value & mask);

				buf_tail
			} else {
				buf
			};

			// Read the rest of data.
			self.register_read_to_slice(reg, buf_tail);
		}
	}
}

/// When sending SPI register address:
/// - MSB = 1 is read operation.
/// - MSB = 0 is write operation.
#[derive(Copy,Clone,Debug)]
#[repr(u8)]
pub enum Mode {
	Write = 0,
	Read  = 1 << 7,
}

/// Helper function for SPI.
///
///  When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3).
#[inline]
pub fn spi_reg_addr(reg: pcd::Reg, mode: Mode) -> u8 {
	mode as u8 | ((reg as u8) << 1)
}


