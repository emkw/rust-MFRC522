/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! MFRC522 registers and named bits.
//!
//! Refer to MFRC522 datasheet for detailed description.

/// MFRC522 registers. Described in chapter 9 of the datasheet.
///
/// When using SPI all addresses need to be shifted one bit left in the
/// "SPI address byte" (section 8.1.2.3) - this has to be done in `MFRC522Bus`
/// implementation
#[derive(Copy,Clone)]
#[cfg_attr(not(feature = "ndebug"), derive(Debug))]
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

pub mod bits {
	//! Re-exports of all defined registers bits.

	pub use super::ComIrq::*;
	pub use super::DivIrq::*;
	pub use super::Error::*;
	pub use super::TxControl::*;
}

#[allow(non_snake_case)]
#[allow(non_upper_case_globals)]
pub mod ComIrq {
	//! ComIrqReg (0x04) bits.

	bitflags! {
		/// Bitflags for ComIrqReg (0x04) bits.
		pub flags ComIrqBits: u8 {
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
}

#[allow(non_snake_case)]
#[allow(non_upper_case_globals)]
pub mod DivIrq {
	//! DivIrqReg (0x05) bits.

	bitflags! {
		/// Bitflags for DivIrqReg (0x05) bits.
		pub flags DivIrqBits: u8 {
			const Set2           = 1 << 7,
			const _bit_05_6      = 1 << 6,
			const _bit_05_5      = 1 << 5,
			const MfinActIRq     = 1 << 4,
			const _bit_05_3      = 1 << 3,
			const CRCIRq         = 1 << 2,
			const _bit_05_1      = 1 << 1,
			const _bit_05_0      = 1 << 0,
		}
	}
}

#[allow(non_snake_case)]
#[allow(non_upper_case_globals)]
pub mod Error {
	//! ErrorReg (0x06) bits.

	bitflags! {
		/// Bitflags for ErrorReg (0x06) bits.
		pub flags ErrorBits: u8 {
			/// Data written into the FIFO buffer by the host during the MFAuthent
			/// command or if during the time between sending and receiving the last
			/// bit on the RF interface.
			const WrErr          = 1 << 7,
			/// Internal temperature sensor detects overheating.
			const TempErr        = 1 << 6,
			/// ErrorReg bit 5 reserved for future use.
			const _bit_06_5      = 1 << 5,
			/// Trying to write data to the FIFO buffer even though it is already full.
			const BufferOvfl     = 1 << 4,
			/// A bit-collision is detected.
			const CollErr        = 1 << 3,
			/// The RxModeReg register's RxCRCEn bit is set and the CRC calculation fails.
			const CRCErr         = 1 << 2,
			/// Parity check failed.
			const ParityErr      = 1 << 1,
			/// Set to logic 1 if the SOF (Start Of Frame) is incorrect.
			const ProtocolErr    = 1 << 0,
		}
	}
}

#[allow(non_snake_case)]
#[allow(non_upper_case_globals)]
pub mod TxControl {
	//! TxControlReg (0x14) bits.

	bitflags! {
		/// Bitflags for TxControlReg (0x14) bits.
		pub flags TxControlBits: u8 {
			/// Output signal on pin TX2 inverted when driver TX2 is enabled
			const InvTx2RFOn     = 1 << 7,
			/// Output signal on pin TX1 inverted when driver TX1 is enabled
			const InvTx1RFOn     = 1 << 6,
			/// Output signal on pin TX2 inverted when driver TX2 is disabled
			const InvTx2RFOff    = 1 << 5,
			const Tx2CW          = 1 << 3,
			/// TxControlReg bit 2 reserved for future use.
			const _bit_14_2      = 1 << 2,
			const Tx2RFEn        = 1 << 1,
			const Tx1RFEn        = 1 << 0,
		}
	}
}
