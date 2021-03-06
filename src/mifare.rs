/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! MIFARE routines.

use super::*;
use super::pcd::reg::bits::*;

impl<'a> MFRC522<'a> {
	/**
	 * Executes the MFRC522 MFAuthent command.
	 *
	 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
	 * The authentication is described in the MFRC522 datasheet
	 * section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
	 *
	 * For use with MIFARE Classic PICCs.
	 *
	 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
	 *
	 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
	 *
	 * All keys are set to FFFFFFFFFFFFh at chip delivery.
	 *
	 * Parameters:
	 * - `command` - picc::Cmd::MF_AUTH_KEY_A or picc::Cmd::MF_AUTH_KEY_B.
	 * - `block_addr` - The block number.
	 * - `key` - Reference to the Crypto1 key to use (6 bytes).
	 * - `uid` - Reference to UID struct. The first 4 bytes of the UID are used.
	 *
	 * Returns: Status::OK on success, Status::??? otherwise. Probably Status::Timeout if you supply the wrong key.
	 */
	pub fn mifare_authenticate(&mut self, command: picc::Cmd, block_addr: u8, key: &picc::mifare::Key, uid: &picc::uid::UID) -> Status {
		let wait_irq = IdleIRq;

		// Build command buffer
		let mut tx_buffer: [u8; 12] = [0; 12];
		tx_buffer[0] = command as u8;
		tx_buffer[1] = block_addr;
		// 6 key bytes.
		tx_buffer[2..2+key.len()].copy_from_slice(key.as_ref());
		// The first 4 bytes of the UID.
		tx_buffer[8..8+4].copy_from_slice(&uid.as_bytes()[..4]);

		// Start the authentication.
		let (status, _) = self.pcd_communicate_with_picc(pcd::Cmd::MFAuthent, wait_irq, tx_buffer.as_ref(), None, &mut 0, 0, false);

		status
	}

	/**
	 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
	 *
	 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
	 *
	 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
	 * The MF0ICU1 returns a NAK for higher addresses.
	 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
	 * For example; if `block_addr` is 03h then pages 03h, 04h, 05h, 06h are returned.
	 * A roll-back is implemented: If `block_addr` is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
	 *
	 * The buffer must be at least 18 bytes because a CRC_A is also returned.
	 * Checks the CRC_A before returning Status::Ok.
	 *
	 * Parameters:
	 * - `block_addr`: MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
	 * - `buffer`: The buffer to store the data in. Must be at least 18 bytes, otherwise `Status::BufferShort` is returned.
	 *
	 * Returns: (status, number_of_bytes_read).
	 *
     * Status is `Status::Ok` on success, `Status::???` otherwise.
	 */
	pub fn mifare_read(&mut self, block_addr: u8, buffer: &mut [u8]) -> (Status, usize) {
		// Sanity check
		if buffer.len() < 18 {
			return (Status::BufferShort, 0);
		}
		// Build command buffer
		let mut tx_buffer: [u8; 4] = [0; 4];
		tx_buffer[0] = picc::Cmd::MF_READ as u8;
		tx_buffer[1] = block_addr;
		let crc_status = self.crc_append(&mut tx_buffer);
		if !crc_status.is_ok() {
			return (crc_status, 0);
		}

		// Transmit the buffer and receive the response, validate CRC_A.
		let (status, nread) = self.pcd_transceive_data(tx_buffer.as_ref(), Some(buffer), &mut 0, 0, false);

		// Card responded, could not read.
		// TODO: check specifications for error codes.
		if nread == 1 && buffer[0] == 0x04 {
			return (Status::Error, nread)
		}

		(status, nread)
	}

	/**
	 * Writes 16 bytes to the active PICC.
	 *
	 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
	 *
	 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
	 * Even though 16 bytes are transferred to the Ultralight PICC, only the first 4 bytes (bytes 0 to 3)
	 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
	 *
	 * Parameters:
	 * - `block_addr`: MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
	 * - `data`: 16 bytes to write to the PICC. If data.len() != 16 error will be returned.
	 *
	 * Returns: `Status::Ok` on success, `Status::???` otherwise.
	 **/
	pub fn mifare_write(&mut self, block_addr: u8, data: &[u8]) -> Status {
		// Sanity check
		if data.len() < 16 {
			return Status::InvalidArg;
		}

		self.mifare_modify(picc::Cmd::MF_WRITE, block_addr, data)
	}

	/**
	 * Helper function for the two-step MIFARE Classic protocol operations write, increment, decrement, and restore.
	 *
	 * Returns: `Status::Ok` on success, `Status::???` otherwise.
	 **/
	pub fn mifare_modify(&mut self, command: picc::Cmd, block_addr: u8, data: &[u8]) -> Status {
		// Mifare Classic protocol requires two communications to perform a write.
		// Step 1: Tell the PICC we want to write to block block_addr.
		let cmd_buffer: [u8; 2] = [
			command as u8,
			block_addr,
		];
		let status = self.pcd_mifare_transceive(&cmd_buffer, false); // Adds CRC_A and checks that the response is MF_ACK.
			if !status.is_ok() {
			return status;
		}

		// Step 2: Transfer the data
		self.pcd_mifare_transceive(data, false) // Adds CRC_A and checks that the response is MF_ACK.
	}
}
