/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! MFRC522 - Library to use RFID 13.56MHz MFRC522 chip.
//!
//! Based on code:
//!
//! * by Dr.Leong (WWW.B2CQSHOP.COM).
//! * by Miguel Balboa (circuitito.com).
//! * by Søren Thing Andersen (access.thing.dk).
//! * by Tom Clement.

#![cfg_attr(not(any(feature = "use_std", feature = "spidev")), no_std)]

#[macro_use]
extern crate log;

#[cfg(any(feature = "use_std", feature = "spidev"))]
extern crate core;

#[cfg(feature = "i2cdev")]
extern crate i2cdev;
#[cfg(feature = "spidev")]
extern crate spidev;

#[macro_use]
mod bitflags;
#[macro_use]
mod macros;
#[macro_use]
mod ndebug;

#[cfg(feature = "self_test")]
pub mod self_test;

pub mod bus;
pub mod host;
#[doc(hidden)]
pub mod mifare;
pub mod pcd;
pub mod picc;

use core::ops::Div;
use core::ops::Mul;

use bus::MFRC522Bus;
use pcd::reg::Reg;
use pcd::reg::bits::*;
use picc::atqa::ATQA;
use picc::uid::UID;

pub struct MFRC522<'a> {
	bus: &'a mut MFRC522Bus,
}

impl<'a> MFRC522<'a> {
	/**
	 * Returns new MFRC522 structure, without performing any PCD initialization nor reset.
	 **/
	#[inline]
	pub fn uninitialized(bus: &'a mut MFRC522Bus) -> Self {
		MFRC522 {
			bus: bus,
		}
	}

	/**
	 * Performs soft reset & initialization and returns new MFRC522 structure.
	 **/
	pub fn init(bus: &'a mut MFRC522Bus) -> Result<Self, Status> {
		let mut mfrc522 = Self::uninitialized(bus);
		try_bus!(mfrc522.pcd_soft_reset());
		try!(mfrc522.pcd_init());

		Ok(mfrc522)
	}

	/**
	 * Initializes the MFRC522 chip.
	 **/
	pub fn pcd_init(&mut self) -> Result<(), Status> {
		// When communicating with a PICC we need a timeout if something goes wrong.
		// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].

		// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds.
		try_bus!(self.register_write(Reg::TMode, 0x80));
		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25µs.
		try_bus!(self.register_write(Reg::TPrescaler, 0xA9));
		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
		try_bus!(self.register_write(Reg::TReloadH, 0x03));
		try_bus!(self.register_write(Reg::TReloadL, 0xE8));

		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
		try_bus!(self.register_write(Reg::TxASK, 0x40));

		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
		try_bus!(self.register_write(Reg::Mode, 0x3D));

		// Enable the transmitter.
		try_bus!(self.pcd_transmitter_on());

		Ok(())
	}

	/**
	 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
	 *
	 * NOTE: The SerialSpeedReg register is reset and therefore the serial data rate is set to 9600.
	 **/
	#[inline]
	pub fn pcd_soft_reset(&mut self) -> bus::Result<()> {
		self.pcd_command(pcd::Cmd::SoftReset)
	}

	/**
	 * Start a command execution on PCD by writing to CommandReg.
	 **/
	#[inline]
	pub fn pcd_command(&mut self, command: pcd::Cmd) -> bus::Result<()> {
		self.register_write(Reg::Command, command as u8)
	}

	/**
	 * Returns bits from ErrorReg.
	 *
	 * ErrorReg[7..0] bits are: WrErr TempErr (reserved)_bit_06_5 BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	 **/
	#[inline]
	pub fn reg_error(&mut self) -> bus::Result<ErrorBits> {
		let bits = try!(self.register_read(Reg::Error));

		Ok(ErrorBits::from_bits_truncate(bits))
	}

	/**
	 * Returns bits from ComIrqReg.
	 *
	 * ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
	 **/
	#[inline]
	pub fn reg_comirq(&mut self) -> bus::Result<ComIrqBits> {
		let bits = try!(self.register_read(Reg::ComIrq));

		Ok(ComIrqBits::from_bits_truncate(bits))
	}

	/**
	 * Returns bits from DivIrqReg.
	 *
	 * DivIrqReg[7..0] bits are: Set2 (reserved)_bit_05_[65] MfinActIRq (reserved)_bit_05_3 CRCIRq (reserved)_bit_05_[10]
	 **/
	#[inline]
	pub fn reg_divirq(&mut self) -> bus::Result<DivIrqBits> {
		let bits = try!(self.register_read(Reg::DivIrq));

		Ok(DivIrqBits::from_bits_truncate(bits))
	}

	/**
	 * Turns the transmitter on by enabling pins TX1 and TX2.
	 * After a reset these pins are disabled.
	 **/
	#[inline]
	pub fn pcd_transmitter_on(&mut self) -> bus::Result<()> {
		let mask = Tx1RFEn | Tx2RFEn;

		self.register_set_bit_mask(Reg::TxControl, mask.bits())
	}

	/**
	 * Turns the transmitter off by disabling pins TX1 and TX2.
	 **/
	#[inline]
	pub fn pcd_transmitter_off(&mut self) -> bus::Result<()> {
		let mask = Tx1RFEn | Tx2RFEn;

		self.register_clear_bit_mask(Reg::TxControl, mask.bits())
	}

	/**
	 * Used to exit the PCD from its authenticated state.
	 *
	 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
	 **/
	#[inline]
	pub fn pcd_stopcrypto1(&mut self) -> bus::Result<()> {
		// Clear MFCrypto1On bit
		// Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
		self.register_clear_bit_mask(Reg::Status2, 0x08)
	}

	/**
	 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
	 **/
	#[inline]
	pub fn pcd_crc_calculate(&mut self, data: &[u8]) -> Result<u16, Status> {
		// Stop any active command.
		try_bus!(self.pcd_command(pcd::Cmd::Idle));
		// Request the CRCIRq interrupt bit.
		try_bus!(self.register_write(Reg::DivIrq, CRCIRq.bits()));
		// FlushBuffer = 1, FIFO initialization
		try_bus!(self.register_set_bit_mask(Reg::FIFOLevel, 0x80));
		// Write data to the FIFO
		try_bus!(self.register_write_slice(Reg::FIFOData, data));
		// Start the calculation
		try_bus!(self.pcd_command(pcd::Cmd::CalcCRC));

		// Wait for the CRC calculation to complete.
		let mut i = 5000;
		loop {
			let divirq = try_bus!(self.reg_divirq());
			if divirq.intersects(CRCIRq) {
				// CRCIRq bit set - calculation done
				break;
			}
			i -= 1;
			if i == 0 {
				// The emergency break. We will eventually terminate on this one after some time.
				// Communication with the MFRC522 might be down.
				return Err(Status::Timeout);
			}
		}
		// Stop calculating CRC for new content in the FIFO.
		try_bus!(self.pcd_command(pcd::Cmd::Idle));

		// Transfer the result from the registers and compose u16.
		let crc_lo = try_bus!(self.register_read(Reg::CRCResultL));
		let crc_hi = try_bus!(self.register_read(Reg::CRCResultH));
		let crc = (crc_hi as u16) << 8 | crc_lo as u16;

		Ok(crc)
	}

	/**
	 * Executes the Transceive command.
	 *
	 * CRC validation can only be done if `recv` buffer is specified.
	 **/
	pub fn pcd_transceive_data(&mut self, send: &[u8], recv: Option<&mut [u8]>,
	                           valid_bits: &mut u8, rx_align: u8, check_crc: bool) -> (Status, usize) {
		let wait_comirq = RxIRq | IdleIRq;

		self.pcd_communicate_with_picc(pcd::Cmd::Transceive, wait_comirq, send, recv, valid_bits, rx_align, check_crc)
	}

	/**
	 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
	 *
	 * CRC validation can only be done if backData and backLen are specified.
	 **/
	pub fn pcd_communicate_with_picc(&mut self, command: pcd::Cmd, wait_comirq: ComIrqBits,
	                                 send: &[u8], recv: Option<&mut [u8]>, last_bits: &mut u8,
	                                 rx_align: u8, check_crc: bool) -> (Status, usize) {
		let mut nread = 0;
		// Prepare values for BitFramingReg
		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
		let bit_framing: u8 = (rx_align << 4) + *last_bits;

		debug!("TX |{}.{}|: {:?}.", send.len(), last_bits, send);

		// Stop any active command
		bus!(self.pcd_command(pcd::Cmd::Idle), return nread);
		// Clear all seven interrupt request bits.
		bus!(self.register_write(Reg::ComIrq, 0x7F), return nread);
		// FlushBuffer = 1, FIFO initialization
		bus!(self.register_set_bit_mask(Reg::FIFOLevel, 0x80), return nread);
		// Write data to the FIFO
		bus!(self.register_write_slice(Reg::FIFOData, send), return nread);
		// Bit adjustments
		bus!(self.register_write(Reg::BitFraming, bit_framing), return nread);
		// Execute the command
		bus!(self.pcd_command(command), return nread);
		if command == pcd::Cmd::Transceive {
			// StartSend=1, transmission of data starts
			bus!(self.register_set_bit_mask(Reg::BitFraming, 0x80), return nread);
		}

		// Wait for the command to complete.
		// In mfrc522::init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
		let mut i = 2000;
		loop {
			let comirq = bus!(self.reg_comirq(), return nread);
			// One of the interrupts that signal success has been set.
			if comirq.intersects(wait_comirq) {
				break;
			}

			// Timer interrupt - nothing received in 25ms
			if comirq.intersects(TimerIRq) {
				debug!("TimerIRq triggered.");
				return (Status::Timeout, nread);
			}

			// The emergency break. If all other conditions fail we will eventually terminate on this one.
			// Communication with the MFRC522 might be down.
			i -= 1;
			if i == 0 {
				warn!("Timeout waiting for TimerIRq.");
				return (Status::Timeout, nread);
			}
		}
	
		// Stop now if any errors except collisions were detected.
		let error = bus!(self.reg_error(), return nread);
		if error.intersects(BufferOvfl | ParityErr | ProtocolErr) {
			return (Status::Error, 0);
		}

		// If the caller wants recv data, get it from the MFRC522.
		if let Some(recv) = recv {
			// Number of bytes in the FIFO
			let n = bus!(self.register_read(Reg::FIFOLevel), return nread) as usize;
			trace!("RX |{}/{}|.", n, recv.len());

			if n > recv.len() {
				return (Status::BufferShort, nread);
			}
			// Get received data from FIFO
			bus!(self.register_read_to_slice(Reg::FIFOData, &mut recv[0..n], rx_align), return nread);
			nread = n;
			// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
			*last_bits = 0x07 & bus!(self.register_read(Reg::Control), return nread);
			debug!("RX |{}.{}|: {:?}.", n, *last_bits, recv);

			// Tell about collisions
			if error.intersects(CollErr) {
				return (Status::Collision, nread);
			}

			// Perform CRC_A validation if requested.
			if check_crc {
				// In this case a MIFARE Classic NAK is not OK.
				if recv.len() == 1 && *last_bits == 4 {
					return (Status::MifareNAK, nread);
				}
				// We need at least the CRC_A value and all 8 bits of the last byte must be received.
				if recv.len() < 2 || *last_bits != 0 {
					return (Status::CRCError, nread);
				}
				// Verify CRC_A - do our own calculation and store the it in crc_buffer.
				let crc = self.crc_calculate(&recv[..recv.len()-2]);
				if let Err(crc_fail) = crc {
					return (crc_fail, nread);
				} else if let Ok(crc) = crc {
					if !Self::u16_le_eq(&recv[recv.len()-2..], crc) {
						return (Status::CRCError, nread);
					}
				}
			}
		}

		(Status::Ok, nread)
	}

	/**
	 * Returns `Some(ATQA)` if a PICC responds to `PICC_CMD_REQA`
	 * and `None` otherwise.
	 *
	 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
	 **/
	#[inline]
	pub fn picc_is_new_card_present(&mut self) -> Option<ATQA> {
		let (status, atqa) = self.picc_reqa();
		match status {
			Status::Ok        |
			Status::Collision => Some(atqa),
			_ => None,
		}
	}

	/**
	 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
	 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
	 * On success:
	 *   - The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT.
	 *     (Figure 7 of the ISO/IEC 14443-3 draft.)
	 *   - The UID size and value of the chosen PICC is returned in *uid along with the SAK.
	 *
	 * A PICC UID consists of 4, 7 or 10 bytes.
	 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
	 *         UID size    Number of UID bytes        Cascade levels        Example of PICC
	 *         ========    ===================        ==============        ===============
	 *         single       4                         1                     MIFARE Classic
	 *         double       7                         2                     MIFARE Ultralight
	 *         triple      10                         3                     Not currently in use?
	 *
	 **/
	pub fn picc_select(&mut self, uid: &mut UID) -> Status {
		// Description of TX buffer structure:
		// Byte 0: SEL              Indicates the Cascade Level: SEL_CL1, SEL_CL2 or SEL_CL3
		// Byte 1: NVB              Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
		// Byte 2: UID-data or CT   See explanation below. CT means Cascade Tag.
		// Byte 3: UID-data
		// Byte 4: UID-data
		// Byte 5: UID-data
		// Byte 6: BCC              Block Check Character - XOR of bytes 2-5
		// Byte 7: CRC_A
		// Byte 8: CRC_A
		// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
		//
		// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
		//  UID size    Cascade level    Byte2    Byte3    Byte4    Byte5
		//  ========    =============    =====    =====    =====    =====
		//  4 bytes     1                uid0     uid1     uid2     uid3
		//  7 bytes     1                  CT     uid0     uid1     uid2
		//              2                uid3     uid4     uid5     uid6
		//  10 bytes    1                  CT     uid0     uid1     uid2
		//              2                  CT     uid3     uid4     uid5
		//              3                uid6     uid7     uid8     uid9
		// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A = 9 bytes
		let mut tx_buffer:  [u8;  9] = [0;  9];
		let mut rx_buffer:  [u8;  5] = [0;  5];
		let mut uid_idx; // The first index in uid_buffer[] that is used in the current Cascade Level.
		let mut uid_selected_bits = 0; // Number of uid bits selected in total.
		let uid_bits_len = uid.bits_len();

		// Sanity checks
		if uid_bits_len > 80 {
			return Status::InvalidArg;
		}

		// Prepare MFRC522
		// ValuesAfterColl=1 => Bits received after collision are cleared.
		bus!(self.register_clear_bit_mask(Reg::Coll, 0x80));

		let mut cascade_level = 1;
		// Repeat Cascade Level loop until we have a complete UID.
		loop {
			let use_cascade_tag;
			// Length of tx including the last, partial byte.
			let mut tx_len;
			// Length of tx including the last, partial byte.
			let mut rx_len;
			// Number of bits in the last, partial byte, on tx/rx.
			let mut last_bits;

			// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
			match cascade_level {
				1 => {
					tx_buffer[0] = picc::Cmd::SEL_CL1 as u8;
					uid_idx = 0;
					// When we know that the UID has more than 4 bytes
					use_cascade_tag = uid_bits_len > (4*8);
				},

				2 => {
					tx_buffer[0] = picc::Cmd::SEL_CL2 as u8;
					uid_idx = 3;
					// When we know that the UID has more than 7 bytes
					use_cascade_tag = uid_bits_len > (7*8);
				},
				
				3 => {
					tx_buffer[0] = picc::Cmd::SEL_CL3 as u8;
					uid_idx = 6;
					// Never used in CL3.
					use_cascade_tag = false;
				},
				
				_ => return Status::Bug,
			}

			// How many UID bits to transmit.
			let mut tx_uid_bits = {
				let remaining_uid_known_bits = uid_bits_len.saturating_sub(8 * uid_idx);

				// At most 32 bits can be transmitted per cascade level.
				core::cmp::min(32, remaining_uid_known_bits)
			};

			trace!("picc_select(): cascade level: {}, uid_bits_len: {} tx_uid_bits: {}", cascade_level, uid_bits_len, tx_uid_bits);

			// UID destination index in tx_buffer[]
			let mut tx_buffer_idx = 2;

			// If we need to use cascade tag, put it in the tx_buffer.
			if use_cascade_tag {
				tx_buffer[tx_buffer_idx] = picc::CT as u8;
				tx_buffer_idx += 1;

				// If we transmit cascade_tag we can only transit at most 24 bits of UID.
				tx_uid_bits = core::cmp::min(24, tx_uid_bits);
			}

			// Copy the transmitted bits from uid[] to tx_buffer[]
			if tx_uid_bits > 0 {
				// The number of bytes needed to represent the known bits for this level.
				let tx_uid_len =
					tx_uid_bits / 8 +
					if tx_uid_bits % 8 > 0 { 1 } else { 0 };

				let uid_bytes = uid.as_ref();
				for i in 0..(tx_uid_len as usize) {
					tx_buffer[tx_buffer_idx as usize] = uid_bytes[uid_idx as usize + i];
					tx_buffer_idx += 1;
				}
			}

			// Now that the uid data has been copied
			// we include the CT (CascadeTag) 8 bits in tx_uid_bits.
			if use_cascade_tag {
				tx_uid_bits += 8;
			}

			// Repeat anti-collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
			'anti_collision: loop {
				// Flag indicating weather we'll be doing SELECT or ANTICOLLISION.
				let select = tx_uid_bits == 32;

				// Set up how many bits and bytes to send and receive.
				if select {
					// SELECT transmits 4 bytes (CT+UID) + 1 byte BCC + 2 bytes CRC = 7 bytes. (+2 bytes SEL + NVB).
					// NVB (Number of Valid Bits): 7 bytes + 0 bits.
					tx_buffer[1] = 0x70;
					// Calculate BCC - Block Check Character.
					tx_buffer[6] = tx_buffer[2] ^ tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5];

					// Calculate CRC_A of first 7 bytes in tx_buffer.
					// and write them to last 2 bytes.
					let crc_status = self.crc_append(&mut tx_buffer[..9]);
					if !crc_status.is_ok() {
						return crc_status;
					}

					// lest bits = 0 => All 8 bits of last bit are used.
					last_bits          = 0;
					// tx_len = 2B (SEL+NVB) + 7B UID
					tx_len             = 2 + 7;
				} else { // This is an ANTICOLLISION.
					last_bits           = tx_uid_bits % 8;
					// Number of whole bytes in the UID part.
					let uid_whole_bytes = tx_uid_bits / 8;
					// Number of whole bytes: 2B (SEL+NVB) + UID whole bytes.
					let tx_whole_bytes  = 2 + uid_whole_bytes;

					// Write NVB (Number of Valid Bits)
					tx_buffer[1]        = (tx_whole_bytes << 4) | last_bits;
					//
 					tx_len              = (tx_whole_bytes + if last_bits != 0 { 1 } else { 0 }) as usize;
				}
				debug!("picc_select(): CL: {}, select: {}, tx_uid_bits: {}, tx_len: {}.", cascade_level, select, tx_uid_bits, tx_len);

				// Set bit adjustments
				// Having a separate binding is overkill. But it makes the next line easier to read.
				let rx_align = last_bits;
				// RxAlign = BitFramingReg[6...4]. TxLastBits = BitFramingReg[2...0]
				bus!(self.register_write(Reg::BitFraming, (rx_align << 4) | last_bits));

				// Transmit the buffer and receive the response.
				let (result, nread) = self.pcd_transceive_data(&tx_buffer[..tx_len], Some(rx_buffer.as_mut()), &mut last_bits, rx_align, false);
				rx_len = nread;

				if result != Status::Ok && result != Status::Collision {
					return result;
				} else {
					// Copy new received UID bits to tx_buffer.
					let rx_uid_offset = if use_cascade_tag { 1 } else { 0 };
					let rx_uid_new = nread
						.saturating_sub(rx_uid_offset)
						.mul(8)
						.saturating_sub(tx_uid_bits as usize)
						.div(8);
					for i in 0..rx_uid_new {
						tx_buffer[tx_buffer_idx] = rx_buffer[rx_uid_offset + i];
						tx_buffer_idx += 1;
					}

					if result == Status::Collision { // More than one PICC in the field => collision.
						// CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
						let value_of_coll_reg = bus!(self.register_read(Reg::Coll));

						if value_of_coll_reg & 0x20 != 0 { // CollPosNotValid
							// Without a valid collision position we cannot continue.
							return Status::Collision;
						}

						// Values 0-31, 0 means bit 32.
						let mut collision_pos = value_of_coll_reg & 0x1F;
						if collision_pos == 0 {
							collision_pos = 32;
						}

						if collision_pos <= tx_uid_bits {
							// No progress - should not happen.
							return Status::Bug;
						}

						// Choose the PICC with the bit set.
						tx_uid_bits          = collision_pos;
						// Collision bit position. The bit to modify
						let bit_pos          = (tx_uid_bits - 1) % 8;
						// Collision byte index. First byte is index 0.
						let byte_idx         = (tx_uid_bits / 8 + 1 + if bit_pos != 0 { 1 } else { 0 }) as usize;
						tx_buffer[byte_idx]  |= 1 << bit_pos;
					} else { // Status::Ok
						if !select { // This was an ANTICOLLISION
							// Copy the found UID bytes from rx_buffer[] to uid[]
							let rx_buffer_uid_index  = if use_cascade_tag { 1 } else { 0 };
							let uid_bytes_to_copy    = if use_cascade_tag { 3 } else { 4 };
							for i in 0..uid_bytes_to_copy as usize {
								uid.as_mut()[uid_idx as usize + i] = rx_buffer[rx_buffer_uid_index + i];
							}
							uid_selected_bits = 8*uid_idx + 8*uid_bytes_to_copy;

							if nread == 5 && last_bits == 0 {
								// Check the returned BCC.
								let bcc_sum = rx_buffer[0] ^ rx_buffer[1] ^ rx_buffer[2] ^ rx_buffer[3] ^ rx_buffer[4];
								if bcc_sum != 0 {
									return Status::BCCError;
								}
							}

							// We now have all 32 bits of the UID in this Cascade Level
							tx_uid_bits = 32;
							// Run 'anti_collision loop again to do the SELECT.
						} else { // This was a SELECT.
							// We continue below, outside the anti_collision loop.
							break 'anti_collision;
						}
					}
				}
			} // 'anti_collision

			// Check response SAK (Select Acknowledge)
			if rx_len != 3 || last_bits != 0 {
				// SAK must be exactly 24 bits (1 byte + CRC_A).
				return Status::Error;
			}

			// Verify CRC_A
			// do our own calculation and store the control in tx_buffer[0..2] - those bytes are not needed anymore.
			let crc = self.crc_calculate(&rx_buffer[0..1]);
			if let Err(crc_fail) = crc {
				return crc_fail;
			} else if let Ok(crc) = crc {
				if !Self::u16_le_eq(&rx_buffer[1..], crc) {
					return Status::CRCError;
				}
			}

			if rx_buffer[0] & 0x04 != 0 { // Cascade bit set - UID not complete yet
				cascade_level += 1;
			} else { // UID complete.
				uid.set_bits_len(uid_selected_bits);
				uid.set_sak(rx_buffer[0]);
				break;
			}
		}

		Status::Ok
	}

	/**
	 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY
	 * and prepare for anticollision or selection. 7 bit frame.
	 *
	 * Beware: When two PICCs are in the field at the same time it often yields Error::Timeout - probably due do bad antenna design.
	 **/
	#[inline]
	pub fn picc_reqa(&mut self) -> (Status, ATQA) {
		self.picc_reqa_or_wupa(picc::Cmd::REQA)
	}

	/**
	 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*)
	 * and prepare for anticollision or selection. 7 bit frame.
	 *
	 * Beware: When two PICCs are in the field at the same time it often yields Error::Timeout - probably due do bad antenna design.
	 **/
	#[inline]
	pub fn picc_wupa(&mut self) -> (Status, ATQA) {
		self.picc_reqa_or_wupa(picc::Cmd::WUPA)
	}

	/**
	 * Transmits a Halt command, Type A. Instructs a PICC in state ACTIVE(*) to go to state HALT.
	 **/
	pub fn picc_hlta(&mut self) -> Status {
		let mut tx_buffer = [0; 4];
		tx_buffer[0] = picc::Cmd::HLTA as u8;
		// tx_buffer[1] <- 0

		// Calculate CRC_A
		// TODO: Investigate if const CRC can be used, or if it must follow the
		//       CRC preset set in mfrc522 register.
		let crc_status = self.crc_append(tx_buffer.as_mut());
		if !crc_status.is_ok() {
			return crc_status;
		}

		// Send the command.
		// The standard says:
		//   If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
		//   HLTA command, this response shall be interpreted as 'not acknowledge'.
		// We interpret that this way: Only STATUS_TIMEOUT is a success.
		let (status, _) = self.pcd_transceive_data(tx_buffer.as_ref(), None, &mut 0_u8, 0, false);
		if status == Status::Timeout {
			return Status::Ok;
		} else if status == Status::Ok { // That is ironically NOT ok in this case ;-)
			return Status::Error;
		}

		status
	}

	/**
	 * Transmits REQA or WUPA commands.
	 *
	 * Beware: When two PICCs are in the field at the same time it often yields Error::Timeout - probably due do bad antenna design.
	 **/
	fn picc_reqa_or_wupa(&mut self, command: picc::Cmd) -> (Status, ATQA) {
		// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte.
		// TxLastBits = BitFramingReg[2..0]
		let mut valid_bits: u8 = 7;
	
		// ValuesAfterColl=1 => Bits received after collision are cleared.
		bus!(self.register_clear_bit_mask(Reg::Coll, 0x80), return ATQA::from(0));
		let data = &[command as u8];
		let mut buffer_atqa = [0_u8; 2];

		// TODO: enable CRC check.
		let (status, nread) = self.pcd_transceive_data(data, Some(buffer_atqa.as_mut()), &mut valid_bits, 0, false);
		if status.is_ok() && (nread != 2 || valid_bits != 0) {
			// ATQA must be exactly 16 bits.
			(Status::Error, ATQA::from(0))
		} else {
			// The ISO/IEC 14443 transfers LSByte first (Little-endian).
			let atqa_bits: u16 = (buffer_atqa[1] as u16) << 8 | buffer_atqa[0] as u16;

			(status, ATQA::from(atqa_bits))
		}
	}

	/**
	 * Calculate CRC.
	 *
	 * Calculates CRC_A, on host or on PCD, controlled at compile-time with
	 * feature `host_crc`.
	 **/
	#[inline]
	pub fn crc_calculate(&mut self, data: &[u8]) -> Result<u16, Status> {
		let ret = if cfg!(feature = "host_crc") {
			let crc = host::crc::crc_iso14443a(data);

			Ok(crc)
		} else {
			self.pcd_crc_calculate(data)
		};
		ndebug!("crc_calculate(): {:?} -> {:?}", data, ret);

		ret
	}

	/**
	 * Calculate CRC of all but last 2 bytes of `buffer`,
	 * and write it to the last 2 bytes of `buffer`.
	 *
	 * Calculates CRC_A, on host or on PCD, controlled at compile-time with
	 * feature `host_crc`.
	 **/
	pub fn crc_append(&mut self, buffer: &mut [u8]) -> Status {
		let split = buffer.len() - 2;
		let (data, crc_buffer) = buffer.split_at_mut(split);
		let crc = self.crc_calculate(data);
		match crc {
			Ok(crc) => {
				crc_buffer[0] = crc as u8;
				crc_buffer[1] = (crc >> 8) as u8;

				Status::Ok
			},

			Err(crc_status) => crc_status,
		}
	}

	/**
	 * Check if little endian bytes match u16.
	 *
	 * Used mainly for crc checks.
	 **/
	#[inline]
	pub fn u16_le_eq(bytes: &[u8], value: u16) -> bool {
		bytes[0] == value as u8 && bytes[1] == (value >> 8) as u8
	}

	/**
	 * Sets the bits given in `mask` in register `reg`.
	 **/
	#[inline]
	pub fn register_set_bit_mask(&mut self, reg: Reg, mask: u8) -> bus::Result<()> {
		let current = try!(self.register_read(reg));
		if (current & mask) != mask {
			try!(self.register_write(reg, current | mask));
		}

		Ok(())
	}

	/**
	 * Clears the bits given in `mask` from register `reg`.
	 **/
	#[inline]
	pub fn register_clear_bit_mask(&mut self, reg: Reg, mask: u8) -> bus::Result<()> {
		let current = try!(self.register_read(reg));
		if (current & mask) != 0x00 {
			try!(self.register_write(reg, current & !mask));
		}

		Ok(())
	}

	/**
	 * Writes byte `value` to register `reg`.
	 **/
	#[inline]
	pub fn register_write(&mut self, reg: Reg, value: u8) -> bus::Result<()> {
		self.bus.register_write(reg, value)
	}

	/**
	 * Writes bytes `values` to register `reg`.
	 **/
	#[inline]
	pub fn register_write_slice(&mut self, reg: Reg, values: &[u8]) -> bus::MultiResult<usize> {
		self.bus.register_write_slice(reg, values)
	}

	/**
	 * Reads byte from register `reg`.
	 **/
	#[inline]
	pub fn register_read(&mut self, reg: Reg) -> bus::Result<u8> {
		self.bus.register_read(reg)
	}

	/**
	 * Reads bytes from register `reg` into slice `values`.
	 **/
	#[inline]
	pub fn register_read_to_slice(&mut self, reg: Reg, buf: &mut [u8], rx_align: u8) -> bus::MultiResult<usize> {
		self.bus.register_read_to_slice_align(reg, buf, rx_align)
	}
}

/// Status codes returned by the functions.
///
/// `Ok` should indicate correct data from all the functions.
/// `Collision` may indicate correct data from some functions.
#[derive(Copy,Clone,PartialEq)]
#[cfg_attr(not(feature = "ndebug"), derive(Debug))]
pub enum Status {
	/// No errors.
	Ok,

	/// Collission detected.
	Collision,
	/// The CRC_A does not match
	CRCError,
	/// The BCC byte check (parity) is not correct.
	BCCError,

	/// Low-level (bus) error in communication.
	///
	/// This is returned when MFRC522Bus implementation
	/// Returns an error.
	BusError,
	/// Timeout in communication.
	///
	/// The reader haven't responded to request within
	/// the time limit.
	Timeout,

	/// Error in communication.
	///
	/// The reader has responded, but the response was
	/// not what was expected.
	Error,
	/// A MIFARE PICC responded with NAK.
	MifareNAK,

	/// Invalid function argument.
	///
	/// Some value may be outside of the supported range.
	InvalidArg,
	/// Receive buffer is not big enough for the response.
	BufferShort,
	/// Internal error in the code. Should not happen ;-)
	Bug,
}

impl Status {

	/// Returns `true` if `Status` is `Status::Ok`
	#[inline]
	pub fn is_ok(&self) -> bool {
		*self == Status::Ok
	}
}
