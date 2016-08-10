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

use pcd::reg::Reg;

/// Bus Result type, for single-byte read/write operations.
pub type Result<T> = ::core::result::Result<T, ()>;

/// Bus Result type, for multi-byte read/write operations.
///
/// Essentially: return `Ok(T)` on success and `Err(T)` on error.
///
/// Typically T will be usize - number of bytes read/written so far.
///
/// Err may pass down as `Status::BusError` from MFRC522 functions.
pub type MultiResult<T> = ::core::result::Result<T, T>;

pub trait MFRC522Bus {
	/// Read single byte from MFRC522 register.
	#[must_use]
	fn register_read(&mut self, reg: Reg) -> Result<u8>;
	/// Write single byte to MFRC522 register.
	#[must_use]
	fn register_write(&mut self, reg: Reg, value: u8) -> Result<()>;

	/// Write multiple bytes to a single MFRC522 register.
	#[must_use]
	fn register_write_slice(&mut self, reg: Reg, values: &[u8]) -> MultiResult<usize> {
		let mut nwrit = 0;
		for byte in values {
			try_map_err!(self.register_write(reg, *byte), nwrit);
			nwrit += 1;
		}

		Ok(nwrit)
	}

	/// Read multiple bytes from a single MFRC522 register.
	#[must_use]
	fn register_read_to_slice(&mut self, reg: Reg, buf: &mut [u8]) -> MultiResult<usize> {
		let mut nread = 0;
		for b in buf {
			*b = try_map_err!(self.register_read(reg), nread);
			nread += 1;
		}

		Ok(nread)
	}

	/// Read multiple bytes from a single MFRC522 register,
	/// only partialy overwriting the first buffer byte.
	#[must_use]
	fn register_read_to_slice_align(&mut self, reg: Reg, buf: &mut [u8], rx_align: u8) -> MultiResult<usize> {
		if buf.len() == 0 {
			return Ok(0)
		}

		let mut nread = 0;
		let buf_tail = if rx_align != 0 {
			let (buf_head, buf_tail) = buf.split_at_mut(1);
			let value = try_map_err!(self.register_read(reg), nread);
			// Only update bit positions rxAlign..7 in buf[0]
			buf_head[0] = rx_align_u8(buf_head[0], value, rx_align);
			nread += 1;

			buf_tail
		} else {
			buf
		};

		if buf_tail.len() > 0 {
			// Read the rest of data.
			nread += try!(self.register_read_to_slice(reg, buf_tail).map_err(|sread| sread + nread))
		}

		Ok(nread)
	}
}

/// When sending SPI register address:
/// - MSB = 1 is read operation.
/// - MSB = 0 is write operation.
#[derive(Copy,Clone)]
#[cfg_attr(not(feature = "ndebug"), derive(Debug))]
#[repr(u8)]
pub enum Mode {
	Write = 0,
	Read  = 1 << 7,
}

/// Helper function doing rx_align on a byte.
///
/// It only updates bit positions 7...`rx_align` in the `byte`
/// with bits from `bits`.
/// TODO: Investigate - this should mimic the original implementation, but
///       shouldn't the lower bytes be updated, rather than higher in the
///       original implementation?
#[inline]
pub fn rx_align_u8(byte: u8, bits: u8, rx_align: u8) -> u8 {
	// Create bit mask for bit positions rx_align..7
	let mask = (!0_u8).checked_shl(rx_align as u32).unwrap_or(0);
	// Apply mask to both current value of byte and the new data in bits.
	(byte & !mask) | (bits & mask)
}

/// Helper function for SPI.
///
///  When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3).
#[inline]
pub fn spi_reg_addr(reg: Reg, mode: Mode) -> u8 {
	mode as u8 | ((reg as u8) << 1)
}
