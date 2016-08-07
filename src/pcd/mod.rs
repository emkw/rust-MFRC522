/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! Definitions for interfacing the *PCD* (Proximity Coupling Device): NXP MFRC522 Contactless Reader IC.

pub mod reg;

/// MFRC522 commands. Described in chapter 10 of the datasheet.
#[derive(Copy,Clone,Debug,Eq,PartialEq)]
#[repr(u8)]
pub enum Cmd {
	/// No action, cancels current command execution.
	Idle                = 0x00,
	/// Stores 25 bytes into the internal buffer.
	Mem                 = 0x01,
	/// Generates a 10-byte random ID number.
	GenerateRandomID    = 0x02,
	/// Activates the CRC coprocessor or performs a self test.
	CalcCRC             = 0x03,
	/// Transmits data from the FIFO buffer.
	Transmit            = 0x04,
	/// No command change, can be used to modify the CommandReg register bits without affecting the command, for example the PowerDown bit.
	NoCmdChange         = 0x07,
	/// Activates the receiver circuits.
	Receive             = 0x08,
	/// Transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission.
	Transceive          = 0x0C,
	/// Performs the MIFARE standard authentication as a reader.
	MFAuthent           = 0x0E,
	/// Resets the MFRC522.
	SoftReset           = 0x0F
}

/// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
/// www.nxp.com/documents/data_sheet/MFRC522.pdf
/// Described in 9.3.3.6 / table 98 of the datasheet at http:
#[allow(non_camel_case_types)]
#[derive(Copy,Clone,Debug)]
#[repr(u8)]
pub enum RxGain {
	/// 000b - 18 dB, minimum.
	Gain18dB            = 0x00 << 4,
	/// 001b - 23 dB.
	Gain23dB            = 0x01 << 4,
	/// 010b - 18 dB, it seems 010b is a duplicate for 000b.
	Gain18dB_2          = 0x02 << 4,
	/// 011b - 23 dB, it seems 011b is a duplicate for 001b.
	Gain23dB_2          = 0x03 << 4,
	/// 100b - 33 dB, average, and typical default.
	Gain33dB            = 0x04 << 4,
	/// 101b - 38 dB.
	Gain38dB            = 0x05 << 4,
	/// 110b - 43 dB.
	Gain43dB            = 0x06 << 4,
	/// 111b - 48 dB, maximum.
	Gain48dB            = 0x07 << 4,
}

pub const RXGAIN_MIN: RxGain = RxGain::Gain18dB;
pub const RXGAIN_AVG: RxGain = RxGain::Gain33dB;
pub const RXGAIN_MAX: RxGain = RxGain::Gain48dB;


