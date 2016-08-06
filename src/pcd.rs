/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! Definitions for interfacing the *PCD* (Proximity Coupling Device): NXP MFRC522 Contactless Reader IC.

/// MFRC522 registers. Described in chapter 9 of the datasheet.
///
/// When using SPI all addresses need to be shifted one bit left in the
/// "SPI address byte" (section 8.1.2.3) - this has to be done in MFRC522Bus
/// implementation
#[derive(Copy,Clone,Debug)]
#[repr(u8)]
pub enum Reg {
	// Page 0: Command and status
	// reserved for future use
	//                    0x00
	/// Starts and stops command execution.
	Command             = 0x01,
	/// Enable and disable interrupt request control bits.
	ComIEn              = 0x02,
	/// Enable and disable interrupt request control bits.
	DivIEn              = 0x03,
	/// Interrupt request bits.
	ComIrq              = 0x04,
	/// Interrupt request bits.
	DivIrq              = 0x05,
	/// Error bits showing the error status of the last command executed.
	Error               = 0x06,
	/// Communication status bits.
	Status1             = 0x07,
	/// Receiver and transmitter status bits.
	Status2             = 0x08,
	/// Input and output of 64 byte FIFO buffer.
	FIFOData            = 0x09,
	/// Number of bytes stored in the FIFO buffer.
	FIFOLevel           = 0x0A,
	/// Level for FIFO underflow and overflow warning.
	WaterLevel          = 0x0B,
	/// Miscellaneous control registers.
	Control             = 0x0C,
	/// Adjustments for bit-oriented frames.
	BitFraming          = 0x0D,
	/// Bit position of the first bit-collision detected on the RF interface.
	Coll                = 0x0E,
	// reserved for future use
	//                    0x0F

	// Page 1: Command
	// reserved for future use
	//                    0x10
	/// defines general modes for transmitting and receiving
	Mode                = 0x11,
	/// defines transmission data rate and framing
	TxMode              = 0x12,
	/// defines reception data rate and framing
	RxMode              = 0x13,
	/// controls the logical behavior of the antenna driver pins TX1 and TX2
	TxControl           = 0x14,
	/// controls the setting of the transmission modulation
	TxASK               = 0x15,
	/// selects the internal sources for the antenna driver
	TxSel               = 0x16,
	/// selects internal receiver settings
	RxSel               = 0x17,
	/// selects thresholds for the bit decoder
	RxThreshold         = 0x18,
	/// Defines demodulator settings.
	Demod               = 0x19,
	// reserved for future use
	//                    0x1A
	// reserved for future use
	//                    0x1B
	/// Controls some MIFARE communication transmit parameters.
	MfTx                = 0x1C,
	/// Controls some MIFARE communication receive parameters.
	MfRx                = 0x1D,
	// reserved for future use
	//                    0x1E
	/// Selects the speed of the serial UART interface.
	SerialSpeed         = 0x1F,

	// Page 2: Configuration
	// reserved for future use
	//                    0x20
	/// Shows MSB value of the CRC calculation.
	CRCResultH          = 0x21,
	/// Shows LSB value of the CRC calculation.
	CRCResultL          = 0x22,
	// reserved for future use
	//                    0x23
	/// Controls the ModWidth setting.
	ModWidth            = 0x24,
	// reserved for future use
	//                    0x25
	/// Configures the receiver gain.
	RFCfg               = 0x26,
	/// Selects the conductance of the antenna driver pins TX1 and TX2 for modulation.
	GsN                 = 0x27,
	/// defines the conductance of the p-driver output during periods of no modulation
	CWGsP               = 0x28,
	/// defines the conductance of the p-driver output during periods of modulation
	ModGsP              = 0x29,
	/// defines settings for the internal timer
	TMode               = 0x2A,
	/// The lower 8 bits of the TPrescaler value. The 4 high bits are in TMode.
	TPrescaler          = 0x2B,
	/// Defines MSB of the 16-bit timer reload value.
	TReloadH            = 0x2C,
	/// Defines LSB of the 16-bit timer reload value.
	TReloadL            = 0x2D,
	/// Shows MSB of 16-bit timer value.
	TCounterValueH      = 0x2E,
	/// Shows LSB of 16-bit timer value.
	TCounterValueL      = 0x2F,

	// Page 3: Test Registers
	// reserved for future use
	//                    0x30
	/// general test signal configuration
	TestSel1            = 0x31,
	/// general test signal configuration
	TestSel2            = 0x32,
	/// enables pin output driver on pins D1 to D7
	TestPinEn           = 0x33,
	/// defines the values for D1 to D7 when it is used as an I/O bus
	TestPinValue        = 0x34,
	/// shows the status of the internal test bus
	TestBus             = 0x35,
	/// controls the digital self test
	AutoTest            = 0x36,
	/// Shows the software version.
	Version             = 0x37,
	/// Controls the pins AUX1 and AUX2.
	AnalogTest          = 0x38,
	/// Defines the test value for TestDAC1.
	TestDAC1            = 0x39,
	/// Defines the test value for TestDAC2.
	TestDAC2            = 0x3A,
	/// Shows the value of ADC I and Q channels.
	TestADC             = 0x3B,
	// reserved for production tests
	//                    0x3C
	// reserved for production tests
	//                    0x3D
	// reserved for production tests
	//                    0x3E
	// reserved for production tests
	//                    0x3F
}

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

// To be decided
#[cfg(disabled)]
pub mod reg {
#![allow(non_upper_case_globals)]
//! Named bits in MFRC522 registers.

	bitflags! {
		/// ComIrqReg (0x04) bits.
		pub flags ComIrqReg: u8 {
			const Set1           = 1 << 7,
			const TxIRq          = 1 << 6,
			const RxIRq          = 1 << 5,
			const IdleIRq        = 1 << 4,
			const HiAlertIRq     = 1 << 3,
			const LoAlertIRq     = 1 << 2,
			const ErrIRq         = 1 << 1,
			const TimerIRq       = 1 << 0,
		}
	}

	bitflags! {
		/// TxControlReg (0x14) bits.
		pub flags TxControlReg: u8 {
			#[doc = "output signal on pin TX2 inverted when driver TX2 is enabled"]
			const InvTx2RFOn     = 1 << 7,
			#[doc = "output signal on pin TX1 inverted when driver TX1 is enabled"]
			const InvTx1RFOn     = 1 << 6,
			#[doc = "output signal on pin TX2 inverted when driver TX2 is disabled"]
			const InvTx2RFOff    = 1 << 5,
			const Tx2CW          = 1 << 3,
			// Bit 2 reserved for future use.
			const _14_Reserved_2 = 1 << 2,
			const Tx2RFEn        = 1 << 1,
			const Tx1RFEn        = 1 << 0,
		}
	}
}
