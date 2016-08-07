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
extern crate bitflags;
#[macro_use]
extern crate log;

#[cfg(any(feature = "use_std", feature = "spidev"))]
extern crate core;

#[cfg(feature = "i2cdev")]
extern crate i2cdev;
#[cfg(feature = "spidev")]
extern crate spidev;

#[cfg(feature = "self_test")]
pub mod self_test;

pub mod bus;
pub mod host;
pub mod pcd;
pub mod picc;

use core::ops::Div;
use core::ops::Mul;

use bus::MFRC522Bus;
use pcd::Reg;
use picc::Uid;

pub struct MFRC522<'a> {
	bus: &'a mut MFRC522Bus,
}

impl<'a> MFRC522<'a> {
	#[inline]
	pub fn uninitialized(bus: &'a mut MFRC522Bus) -> Self {
		MFRC522 {
			bus: bus,
		}
	}

	/**
	 * Initializes the MFRC522 chip.
	 **/
	pub fn init(bus: &'a mut MFRC522Bus) -> Self {
		let mut mfrc522 = Self::uninitialized(bus);
		mfrc522.pcd_soft_reset();
		mfrc522.pcd_init();

		mfrc522
	}

	pub fn pcd_init(&mut self) {
		// When communicating with a PICC we need a timeout if something goes wrong.
		// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].

		// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds.
		self.register_write(Reg::TMode, 0x80);
		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25µs.
		self.register_write(Reg::TPrescaler, 0xA9);
		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
		self.register_write(Reg::TReloadH, 0x03);
		self.register_write(Reg::TReloadL, 0xE8);

		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
		self.register_write(Reg::TxASK, 0x40);

		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
		self.register_write(Reg::Mode, 0x3D);

		// Enable the transmitter.
		self.pcd_transmitter_on();
	}

	/**
	 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
	 *
	 * NOTE: The SerialSpeedReg register is reset and therefore the serial data rate is set to 9600.
	 **/
	#[inline]
	pub fn pcd_soft_reset(&mut self) {
		self.pcd_command(pcd::Cmd::SoftReset);
	}

	/**
	 * Turns the transmitter on by enabling pins TX1 and TX2.
	 * After a reset these pins are disabled.
	 **/
	#[inline]
	pub fn pcd_transmitter_on(&mut self) {
		self.register_set_bit_mask(Reg::TxControl, 0x03);
	}

	/**
	 * Turns the transmitter off by disabling pins TX1 and TX2.
	 **/
	#[inline]
	pub fn pcd_transmitter_off(&mut self) {
		self.register_clear_bit_mask(Reg::TxControl, 0x03);
	}

	/**
	 * Used to exit the PCD from its authenticated state.
	 *
	 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
	 **/
	#[inline]
	pub fn pcd_stopcrypto1(&mut self) {
		// Clear MFCrypto1On bit
		// Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
		self.register_clear_bit_mask(Reg::Status2, 0x08);
	}

	/**
	 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
	 **/
	#[inline]
	pub fn pcd_crc_calculate(&mut self, data: &[u8], output: &mut [u8]) -> Status {
		debug_assert!(output.len() >= 2);

		// Stop any active command.
		self.pcd_command(pcd::Cmd::Idle);
		// Request the CRCIRq interrupt bit.
		self.register_write(Reg::DivIrq, 0x04);
		// FlushBuffer = 1, FIFO initialization
		self.register_set_bit_mask(Reg::FIFOLevel, 0x80);
		// Write data to the FIFO
		self.register_write_slice(Reg::FIFOData, data);
		// Start the calculation
		self.pcd_command(pcd::Cmd::CalcCRC);

		// Wait for the CRC calculation to complete.
		let mut i = 5000;
		loop {
			// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
			let irq = self.register_read(Reg::DivIrq);
			if irq & 0x04 != 0 {
				// CRCIRq bit set - calculation done
				break;
			}
			i -= 1;
			if i == 0 {
				// The emergency break. We will eventually terminate on this one after some time.
				// Communication with the MFRC522 might be down.
				return Status::Timeout;
			}
		}
		// Stop calculating CRC for new content in the FIFO.
		self.pcd_command(pcd::Cmd::Idle);
		// Transfer the result from the registers to the result buffer
		output[0] = self.register_read(Reg::CRCResultL);
		output[1] = self.register_read(Reg::CRCResultH);

		// unimplemented
		Status::Ok
	}

	#[inline]
	pub fn pcd_command(&mut self, command: pcd::Cmd) {
		self.register_write(Reg::Command, command as u8)
	}

	/**
	 * Executes the Transceive command.
	 *
	 * CRC validation can only be done if `recv` buffer is specified.
	 **/
	pub fn pcd_transceive_data(&mut self, send: &[u8], recv: Option<&mut [u8]>,
	                           valid_bits: &mut u8, rx_align: u8, check_crc: bool) -> (Status, usize) {
		// RxIRQ and IdleIRQ
		let wait_irq: u8 = 0x30;

		self.pcd_communicate_with_picc(pcd::Cmd::Transceive, wait_irq, send, recv, valid_bits, rx_align, check_crc)
	}

	/**
	 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
	 *
	 * CRC validation can only be done if backData and backLen are specified.
	 **/
	pub fn pcd_communicate_with_picc(&mut self, command: pcd::Cmd, wait_irq: u8,
	                                 send: &[u8], recv: Option<&mut [u8]>, last_bits: &mut u8,
	                                 rx_align: u8, check_crc: bool) -> (Status, usize) {
		let mut nread = 0;
		// Prepare values for BitFramingReg
		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
		let bit_framing: u8 = (rx_align << 4) + *last_bits;

		debug!("TX |{}.{}|: {:?}.", send.len(), last_bits, send);

		// Stop any active command
		self.pcd_command(pcd::Cmd::Idle);
		// Clear all seven interrupt request bits.
		self.register_write(Reg::ComIrq, 0x7F);
		// FlushBuffer = 1, FIFO initialization
		self.register_set_bit_mask(Reg::FIFOLevel, 0x80);
		// Write data to the FIFO
		self.register_write_slice(Reg::FIFOData, send);
		// Bit adjustments
		self.register_write(Reg::BitFraming, bit_framing);
		// Execute the command
		self.pcd_command(command);
		if command == pcd::Cmd::Transceive {
			// StartSend=1, transmission of data starts
			self.register_set_bit_mask(Reg::BitFraming, 0x80);
		}

		// Wait for the command to complete.
		// In mfrc522::init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
		let mut i = 2000;
		loop {
			// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
			let n = self.register_read(Reg::ComIrq);
			// One of the interrupts that signal success has been set.
			if n & wait_irq != 0 {
				break;
			}

			// Timer interrupt - nothing received in 25ms
			if n & 0x01 != 0 {
				return (Status::Timeout, nread);
			}

			// The emergency break. If all other conditions fail we will eventually terminate on this one.
			// Communication with the MFRC522 might be down.
			i -= 1;
			if i == 0 {
				return (Status::Timeout, nread);
			}
		}
	
		// Stop now if any errors except collisions were detected.
		// ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
		let error_reg_value = self.register_read(Reg::Error);
		// BufferOvfl ParityErr ProtocolErr
		if error_reg_value & 0x13 != 0 {
			return (Status::Error, 0);
		}

		// If the caller wants recv data, get it from the MFRC522.
		if let Some(recv) = recv {
			// Number of bytes in the FIFO
			let n = self.register_read(Reg::FIFOLevel) as usize;
			trace!("RX |{}/{}|.", n, recv.len());

			if n > recv.len() {
				return (Status::BufferShort, nread);
			}
			// Get received data from FIFO
			self.register_read_to_slice(Reg::FIFOData, &mut recv[0..n], rx_align);
			nread = n;
			// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
			*last_bits = self.register_read(Reg::Control) & 0x07;
			debug!("RX |{}.{}|: {:?}.", n, *last_bits, recv);

			// Tell about collisions
			// CollErr
			if error_reg_value & 0x08 != 0 {
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
				let mut crc_buffer: [u8; 2] = [0; 2];
				let status = self.crc_calculate(&recv[..recv.len()-2], &mut crc_buffer[..]);
				if status != Status::Ok {
					return (status, nread);
				}

				if recv[recv.len()-2] != crc_buffer[0] || recv[recv.len()-1] != crc_buffer[1] {
					return (Status::CRCError, nread);
				}
			}
		}

		(Status::Ok, nread)
	}

	/**
	 * Returns `true` if a PICC responds to `PICC_CMD_REQA`.
	 *
	 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
	 **/
	pub fn picc_is_new_card_present(&mut self) -> bool {
		let mut buffer_atqa: [u8; 2] = [0; 2];
		let status = self.picc_reqa(&mut buffer_atqa);

		match status {
			Status::Ok        |
			Status::Collision => true,
			_ => false,
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
	pub fn picc_select(&mut self, uid: &mut Uid) -> Status {
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
		self.register_clear_bit_mask(Reg::Coll, 0x80);

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
					let (crc_data, crc_result) = tx_buffer.split_at_mut(7);
					let result = self.crc_calculate(crc_data, crc_result);
					if result != Status::Ok {
						return result;
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
				self.register_write(Reg::BitFraming, (rx_align << 4) | last_bits);

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
						let value_of_coll_reg = self.register_read(Reg::Coll);

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
									return Status::Error;
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
			let result = self.crc_calculate(&rx_buffer[0..1], &mut tx_buffer[0..2]);
			if result != Status::Ok {
				return result;
			} else if tx_buffer[0] != rx_buffer[1] || tx_buffer[1] != rx_buffer[2] {
				return Status::CRCError;
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
	pub fn picc_reqa(&mut self, buffer: &mut [u8; 2]) -> Status {
		self.picc_reqa_or_wupa(picc::Cmd::REQA, buffer)
	}

	/**
	 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*)
	 * and prepare for anticollision or selection. 7 bit frame.
	 *
	 * Beware: When two PICCs are in the field at the same time it often yields Error::Timeout - probably due do bad antenna design.
	 **/
	#[inline]
	pub fn picc_wupa(&mut self, buffer: &mut [u8; 2]) -> Status {
		self.picc_reqa_or_wupa(picc::Cmd::WUPA, buffer)
	}

	/**
	 * Transmits REQA or WUPA commands.
	 *
	 * Beware: When two PICCs are in the field at the same time it often yields Error::Timeout - probably due do bad antenna design.
	 **/
	pub fn picc_reqa_or_wupa(&mut self, command: picc::Cmd, buffer: &mut [u8; 2]) -> Status {
		// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte.
		// TxLastBits = BitFramingReg[2..0]
		let mut valid_bits: u8 = 7;
	
		// ValuesAfterColl=1 => Bits received after collision are cleared.
		self.register_clear_bit_mask(Reg::Coll, 0x80);
		let data = &[command as u8];

		// TODO: enable CRC check.
		let (status, _) = self.pcd_transceive_data(data, Some(&mut buffer[..]), &mut valid_bits, 0, false);

		if status == Status::Ok && valid_bits != 0 {
			// ATQA must be exactly 16 bits.
			Status::Error
		} else {
			status
		}
	}

	/**
	 * Calculate CRC.
	 *
	 * Calculates CRC_A, on host or on PCD, controlled at compile-time with
	 * feature `host_crc`.
	 **/
	#[inline]
	pub fn crc_calculate(&mut self, data: &[u8], output: &mut [u8]) -> Status {
		let status = if cfg!(feature = "host_crc") {
			let crc = host::crc::crc_iso14443a(data);
			output[0] = crc as u8;
			output[1] = (crc >> 8) as u8;

			Status::Ok
		} else {
			self.pcd_crc_calculate(data, output)
		};
		debug!("crc_calculate(): {:?}: {:?} -> {:?}", status, data, &output[0..2]);

		status
	}

	/**
	 * Sets the bits given in `mask` in register `reg`.
	 **/
	#[inline]
	pub fn register_set_bit_mask(&mut self, reg: Reg, mask: u8) {
		let current = self.register_read(reg);
		if (current & mask) != mask {
			self.register_write(reg, current | mask);
		}
	}

	/**
	 * Clears the bits given in `mask` from register `reg`.
	 **/
	#[inline]
	pub fn register_clear_bit_mask(&mut self, reg: Reg, mask: u8) {
		let current = self.register_read(reg);
		if (current & mask) != 0x00 {
			self.register_write(reg, current & !mask);
		}
	}

	/**
	 * Writes byte `value` to register `reg`.
	 **/
	#[inline]
	pub fn register_write(&mut self, reg: Reg, value: u8) {
		self.bus.register_write(reg, value);
	}

	/**
	 * Writes bytes `values` to register `reg`.
	 **/
	#[inline]
	pub fn register_write_slice(&mut self, reg: Reg, values: &[u8]) {
		self.bus.register_write_slice(reg, values);
	}

	/**
	 * Reads byte from register `reg`.
	 **/
	#[inline]
	pub fn register_read(&mut self, reg: Reg) -> u8 {
		self.bus.register_read(reg)
	}

	/**
	 * Reads bytes from register `reg` into slice `values`.
	 **/
	#[inline]
	pub fn register_read_to_slice(&mut self, reg: Reg, buf: &mut [u8], rx_align: u8) {
		self.bus.register_read_to_slice_align(reg, buf, rx_align)
	}
}

/// Status codes returned by the functions.
#[derive(Copy,Clone,Debug,PartialEq)]
pub enum Status {
	Ok,
	/// Error in communication.
	Error,
	/// Collission detected.
	Collision,
	/// Timeout in communication.
	Timeout,
	/// A buffer is not big enough.
	BufferShort,
	/// Internal error in the code. Should not happen ;-)
	Bug,
	/// Invalid argument.
	InvalidArg,
	/// The CRC_A does not match
	CRCError,
	/// A MIFARE PICC responded with NAK.
	MifareNAK,
}
