/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! Definitions for interfacing *PICC* (Proximity Integrated Circuit Card) - A card or tag using the ISO 14443A interface, eg. Mifare or NTAG203.

pub mod uid;
pub mod atqa;

pub use self::uid::UID;
pub use self::atqa::ATQA;

/// Cascade Tag Type A.
/// Used during anti collision.
pub const CT: u8 = 0x88;

/// Commands sent to the PICC.
#[allow(non_camel_case_types)]
#[derive(Copy,Clone)]
#[cfg_attr(not(feature = "ndebug"), derive(Debug))]
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

/// PICC types we can detect.
#[allow(non_camel_case_types)]
#[derive(Copy,Clone,Eq,PartialEq)]
#[cfg_attr(not(feature = "ndebug"), derive(Debug))]
pub enum Type {
	/// Unknown PICC type
	Unknown,
	/// SAK indicates UID is not complete
	Incomplete,
	/// PICC compliant with ISO/IEC 14443-4
	ISO_14443_4,
	/// PICC compliant with ISO/IEC 18092 (NFC)
	ISO_18092,
	/// MIFARE Classic protocol, 320 bytes
	MIFARE_MINI,
	/// MIFARE Classic protocol, 1KB
	MIFARE_1K,
	/// MIFARE Classic protocol, 4KB
	MIFARE_4K,
	/// MIFARE Ultralight or Ultralight C
	MIFARE_UL,
	/// MIFARE Plus
	MIFARE_PLUS,
	/// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	TNP3XXX,
}

impl Type {

	/// Get picc::Type from SAK byte.
	pub fn from_sak(sak: u8) -> Self {
		use self::Type::*;

		// http://www.nxp.com/documents/application_note/AN10833.pdf
		// 3.2 Coding of Select Acknowledge (SAK)
		// ignore 8-bit (iso14443 starts with LSBit = bit 1)
		// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
		match sak & 0x7F {
			0x04 => Incomplete,
			0x09 => MIFARE_MINI,
			0x08 => MIFARE_1K,
			0x18 => MIFARE_4K,
			0x00 => MIFARE_UL,

			0x10 |
			0x11 => MIFARE_PLUS,

			0x20 => ISO_14443_4,
			0x40 => ISO_18092,
			0x01 => TNP3XXX,

			_ => Unknown,
		}
	}
}

pub mod mifare {
	//! MIFARE PICC definitions.

	/// A Mifare Crypto1 key is 6 bytes.
	pub const KEY_LEN: usize = 6;

	/// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
	pub const ACK: u8 = 0xA;

	/// Default key on factory outlet.
	pub const FACTORY_KEY: Key = Key([0xFF; KEY_LEN]);

	/// MIFARE Crypto1 Key.
	#[derive(Copy,Clone,Eq,PartialEq)]
	#[cfg_attr(not(feature = "ndebug"), derive(Debug))]
	pub struct Key([u8; KEY_LEN]);

	impl Default for Key {
		#[inline]
		fn default() -> Self {
			Key([0; KEY_LEN])
		}
	}

	impl Key {
		#[inline]
		pub fn len(&self) -> usize {
			KEY_LEN
		}
	}

	impl AsRef<[u8]> for Key {
		#[inline]
		fn as_ref(&self) -> &[u8] {
			self.0.as_ref()
		}
	}

	impl AsMut<[u8]> for Key {
		#[inline]
		fn as_mut(&mut self) -> &mut [u8] {
			self.0.as_mut()
		}
	}

	impl From<[u8; 6]> for Key {
		#[inline]
		fn from(bytes: [u8; 6]) -> Self {
			Key(bytes)
		}
	}

	/// Constructs `Key` from 48 lower bits (6 lower bytes) of `u64`.
	impl From<u64> for Key {
		fn from(bits: u64) -> Self {
			let bytes = [
				(bits >> 40) as u8,
				(bits >> 32) as u8,
				(bits >> 24) as u8,
				(bits >> 16) as u8,
				(bits >>  8) as u8,
				bits as u8
			];

			Key::from(bytes)
		}
	}

	#[cfg(test)]
	mod tests {
		use super::Key;

		#[test]
		fn test_key_from_u48() {
			let key_bits: u64 = 0x112233445566;
			let key = Key::from(key_bits);

			assert_eq!(key.as_ref(), &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
		}

		#[test]
		fn test_key_from_u64() {
			let key_bits: u64 = 0x1122334455667788;
			let key = Key::from(key_bits);

			assert_eq!(key.as_ref(), &[0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
		}
	}
}
