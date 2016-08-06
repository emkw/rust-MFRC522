/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! Definitions for interfacing *PICC* (Proximity Integrated Circuit Card) - A card or tag using the ISO 14443A interface, eg. Mifare or NTAG203.

/// Cascade Tag. Used during anti collision.
pub const CT: u8 = 0x88;

/// Commands sent to the PICC.
#[allow(non_camel_case_types)]
#[derive(Copy,Clone,Debug)]
#[repr(u8)]
pub enum Cmd {
	/// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	/// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	REQA			= 0x26,
	/// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	WUPA			= 0x52,
	/// Anti collision/Select, Cascade Level 1
	SEL_CL1			= 0x93,
	/// Anti collision/Select, Cascade Level 2
	SEL_CL2			= 0x95,
	/// Anti collision/Select, Cascade Level 3
	SEL_CL3			= 0x97,
	/// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	HLTA			= 0x50,
	/// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	///
	/// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.	///
	/// The read/write commands can also be used for MIFARE Ultralight.
	///
	/// Perform authentication with Key A
	MF_AUTH_KEY_A	= 0x60,
	/// Perform authentication with Key B
	MF_AUTH_KEY_B	= 0x61,
	/// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	MF_READ			= 0x30,
	/// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	MF_WRITE		= 0xA0,
	/// Decrements the contents of a block and stores the result in the internal data register.
	MF_DECREMENT	= 0xC0,
	/// Increments the contents of a block and stores the result in the internal data register.
	MF_INCREMENT	= 0xC1,
	/// Reads the contents of a block into the internal data register.
	MF_RESTORE		= 0xC2,
	/// Writes the contents of the internal data register to a block.
	MF_TRANSFER		= 0xB0,
	/// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	///
	/// The `MF_READ` and `MF_WRITE` can also be used for MIFARE Ultralight.
	/// Writes one 4 byte page to the PICC.
	UL_WRITE		= 0xA2,
}

/// Uid is either 4, 7 or 10 bytes.
#[derive(Copy,Clone,Debug)]
pub struct Uid {
	/// The UID
	uid: [u8; 10],
	/// UID length is bytes
	len: u8,
	/// This represents how many
	/// bits of a last byte are valid.
	/// 0 means all bits are valid.
	last_bits: u8,
	/// Value of SAK byte
	sak: u8,
}

impl Default for Uid {
	#[inline]
	fn default() -> Self {
		Uid {
			uid: [0; 10],
			len: 0,
			last_bits: 0,
			sak: 0,
		}
	}
}

impl Uid {
	#[inline]
	pub fn len(&self) -> u8 {
		self.len
	}

	pub fn bits_len(&self) -> u8 {
		if self.len == 0 {
			0
		} else {
			8 * self.len
			+ self.last_bits
			- if self.last_bits == 0 { 0 } else { 8 }
		}
	}

	pub fn set_bits_len(&mut self, bits_len: u8) {
		assert!(bits_len <= 80);
		if bits_len == 0 {
			self.len = 0;
			self.last_bits = 0;
		} else {
			self.last_bits = bits_len % 8;
			self.len = bits_len/8 + if self.last_bits != 0 { 1 } else { 0 };
		}
	}

	#[inline]
	pub fn last_bits(&self) -> u8 {
		self.last_bits
	}

	#[inline]
	pub fn sak(&self) -> u8 {
		self.sak
	}

	#[inline]
	pub fn set_sak(&mut self, value: u8) {
		self.sak = value;
	}

	#[inline]
	pub fn as_bytes(&self) -> &[u8] {
		&self.uid[..self.len as usize]
	}

	#[inline]
	pub fn clear(&mut self) {
		for i in 0..self.uid.len() {
			self.uid[i] = 0;
		}
		self.len = 0;
		self.last_bits = 0;
		self.sak = 0;
	}
}

impl AsRef<[u8]> for Uid {
	#[inline]
	fn as_ref(&self) -> &[u8] {
		self.uid.as_ref()
	}
}

impl AsMut<[u8]> for Uid {
	#[inline]
	fn as_mut(&mut self) -> &mut [u8] {
		self.uid.as_mut()
	}
}

impl From<[u8; 4]> for Uid {
	#[inline]
	fn from(bytes: [u8; 4]) -> Self {
		bytes.as_ref().into()
	}
}

impl From<[u8; 7]> for Uid {
	#[inline]
	fn from(bytes: [u8; 7]) -> Self {
		bytes.as_ref().into()
	}
}

impl From<[u8; 10]> for Uid {
	#[inline]
	fn from(bytes: [u8; 10]) -> Self {
		Uid {
			uid: bytes,
			len: 10,
			last_bits: 0,
			sak: 0,
		}
	}
}

impl<'a> From<&'a [u8]> for Uid {
	fn from(bytes: &'a [u8]) -> Self {
		let mut ret = Uid {
			uid: [0; 10],
			len: ::core::cmp::min(10, bytes.len()) as u8,
			last_bits: 0,
			sak: 0,
		};
		for i in 0..ret.len as usize {
			ret.uid[i] = bytes[i];
		}

		ret
	}
}
