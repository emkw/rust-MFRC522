/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! PICC UID (Unique Identifier, Type A).

/// UID struct.
///
/// UID is either 4, 7 or 10 bytes.
#[derive(Copy,Clone)]
#[cfg_attr(not(feature = "ndebug"), derive(Debug))]
pub struct UID {
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

impl Default for UID {
	#[inline]
	fn default() -> Self {
		UID {
			uid: [0; 10],
			len: 0,
			last_bits: 0,
			sak: 0,
		}
	}
}

impl UID {
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
	pub fn picc_type(&self) -> super::Type {
		super::Type::from_sak(self.sak)
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

impl AsRef<[u8]> for UID {
	#[inline]
	fn as_ref(&self) -> &[u8] {
		self.uid.as_ref()
	}
}

impl AsMut<[u8]> for UID {
	#[inline]
	fn as_mut(&mut self) -> &mut [u8] {
		self.uid.as_mut()
	}
}

impl From<[u8; 4]> for UID {
	#[inline]
	fn from(bytes: [u8; 4]) -> Self {
		bytes.as_ref().into()
	}
}

impl From<[u8; 7]> for UID {
	#[inline]
	fn from(bytes: [u8; 7]) -> Self {
		bytes.as_ref().into()
	}
}

impl From<[u8; 10]> for UID {
	#[inline]
	fn from(bytes: [u8; 10]) -> Self {
		UID {
			uid: bytes,
			len: 10,
			last_bits: 0,
			sak: 0,
		}
	}
}

impl<'a> From<&'a [u8]> for UID {
	fn from(bytes: &'a [u8]) -> Self {
		let mut ret = UID {
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
