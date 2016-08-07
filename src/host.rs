/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/// CRC_A routine enabled by feature `host_crc`.
pub mod crc {
	/// CRC_A preset value used.
	#[cfg(feature = "host_crc")]
	pub const CRC_PRESET: u16 = 0x6363;

	/// Calculate CRC_A.
	#[cfg(feature = "host_crc")]
	pub fn crc_iso14443a(data: &[u8]) -> u16 {
		data.iter()
			.fold(CRC_PRESET, |crc, byte| {
				let mut b = *byte;
				b = b ^ (crc as u8);
				b = b ^ (b << 4);

				(crc >> 8) ^ ((b as u16) << 8) ^ ((b as u16) << 3) ^ ((b as u16) >> 4)
		})
	}

	/// Dummy calculate CRC_A, for compilation without `host_crc` feature.
	/// Without `host_crc` this always panic.
	#[cfg(not(feature = "host_crc"))]
	#[inline(always)]
	pub fn crc_iso14443a(_data: &[u8]) -> u16 {
		unimplemented!()
	}

	#[cfg(all(test, feature = "host_crc"))]
	mod tests {
		use super::crc_iso14443a;

		#[test]
		fn test_preset() {
			let value  = [0x63, 0x63];
			let expect = 0x0000;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_00_1() {
			let value  = [0x00_u8; 1];
			let expect = 0x51fe;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_85_1() {
			let value  = [0x85_u8; 1];
			let expect = 0x825b;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_aa_1() {
			let value  = [0xaa_u8; 1];
			let expect = 0x5bae;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_ff_1() {
			let value  = [0xff_u8; 1];
			let expect = 0x5e86;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_00_256() {
			let value  = [0x00_u8; 256];
			let expect = 0x2537;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_ff_256() {
			let value  = [0xff_u8; 256];
			let expect = 0xc66f;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_full_range() {
			let mut value = [0x00_u8; 256];
			for i in 0..256 {
				value[i] = i as u8;
			}
			let expect = 0xfd76;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}

		#[test]
		fn test_full_range_rev() {
			let mut value = [0x00_u8; 256];
			for i in 0..256 {
				value[i] = (256-i) as u8;
			}
			let expect = 0xefad;
			let result = crc_iso14443a(&value);

			assert!(result == expect, "assertion 0x{:04x} == 0x{:04x} failed", result, expect);
		}
	}
}
