/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! PICC ATQA (Answer To Request acc. to ISO/IEC 14443-4).

/// ATQA struct.
pub struct ATQA(u16);

impl ATQA {
	/// Get ATQA bits.
	pub fn bits(&self) -> u16 {
		self.0
	}
}
