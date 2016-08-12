/// This example needs features: spidev
extern crate spidev;
extern crate mfrc522;

extern crate env_logger;

mod bus_spidev;
use bus_spidev::spi_open;

use std::io;
use std::thread;
use std::time::Duration;

use mfrc522::MFRC522;
use mfrc522::picc;
use mfrc522::picc::mifare;

static USAGE: &'static str = "Usage: mifare_write <block> [text]";

fn authenticate_and_write(mfrc522: &mut MFRC522, uid: &picc::UID) -> bool {
	let key = mifare::FACTORY_KEY;

	let block:  u8 = std::env::args().nth(1).expect(USAGE).parse().expect("Block number invalid.");
	let text       = std::env::args().nth(2).unwrap_or(String::from(""));

	if text.len() > 16 {
		println!("Text can be maximum 16 bytes long.");
		return false;
	}

	// Do not try to write anything to the sector trailer block.
	let mut write = false;
	if block % 4 == 3 {
		println!("Do not write arbitrary text to block which is sector trailer!");
	} else {
		write = true;
	}

	let auth_status = mfrc522.mifare_authenticate(picc::Cmd::MF_AUTH_KEY_A, block, &key, uid);
	if !auth_status.is_ok() {
		println!("MIFARE authentication failed: {:?}.", auth_status);
		return false;
	}

	if write && !text.is_empty() {
		println!("Writing block {}.", block);
		let mut wr_buffer: [u8; 16] = [0; 16];
		wr_buffer[..text.len()].copy_from_slice(text.as_ref());

		let wr_status = mfrc522.mifare_write(block, wr_buffer.as_ref());
		if wr_status.is_ok() {
			println!("Succesfuly written block {}.", block);
		} else {
			println!("Could not write block {}: {:?}.", block, wr_status);
		}
	}

	let mut buffer = [0_u8; 18];
	let (rd_status, nread) = mfrc522.mifare_read(block, &mut buffer);
	if rd_status.is_ok() && nread > 0 {
		println!("Reading back block {}: {:?}: {:?}.", block, rd_status, &buffer[..nread]);
	} else {
		println!("Could not read block {}: {:?}.", block, rd_status);
		return false;
	}

	true
}

fn example() -> io::Result<()> {
	let mut bus = try!(spi_open("/dev/spidev0.0"));
	let mut mfrc522 = MFRC522::init(&mut bus).expect("MFRC522 Initialization failed.");

	let mut uid = picc::UID::default();
	loop {
		let new_card = mfrc522.picc_is_new_card_present();
		if let Some(atqa) = new_card {
			println!("New card detected. ATQA: {:04x}", atqa.bits());

			let status = mfrc522.picc_select(&mut uid);
			println!("Select: {:?} {:?}", status, uid);
			if status.is_ok() {
				println!("Card UID: {:?} | Type: {:?}", uid.as_bytes(), uid.picc_type());

				if authenticate_and_write(&mut mfrc522, &uid) {
					let halt_status = mfrc522.picc_hlta();
					println!("Halt: {:?}", halt_status);
					mfrc522.pcd_stopcrypto1().expect("MFRC522: pcd_stopcrypto1() failed.");

					break;
				}
			}

			uid.clear();
		}

		thread::sleep(Duration::from_millis(500));
	}

	Ok(())
}

pub fn main() {
	env_logger::init().unwrap_or_else(|e| println!("Could not init env_logger: {:?}.", e));

	match example() {
		Ok(_)  => println!("Done."),
		Err(e) => println!("Error: {:?}.", e),
	}
}
