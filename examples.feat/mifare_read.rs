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
use mfrc522::picc::UID;
use mfrc522::picc::mifare;

fn authenticate_and_read_block_zero(mfrc522: &mut MFRC522, uid: &UID) -> bool {
	let key = mifare::FACTORY_KEY;

	let auth_status = mfrc522.mifare_authenticate(picc::Cmd::MF_AUTH_KEY_A, 0, &key, uid);
	if !auth_status.is_ok() {
		println!("MIFARE authentication failed: {:?}", auth_status);
		return false;
	}

	let mut buffer = [0_u8; 18];
	let (read_status, nread) = mfrc522.mifare_read(0, &mut buffer);
	if read_status.is_ok() && nread > 0 {
		println!("Block 0 {:?}: {:?}", read_status, &buffer[..nread]);
	} else {
		println!("Could not read anything from block 0: {:?}", read_status);
		return false;
	}

	true
}

fn example() -> io::Result<()> {
	let mut bus = try!(spi_open("/dev/spidev0.0"));
	let mut mfrc522 = MFRC522::init(&mut bus).expect("MFRC522 Initialization failed");

	let mut uid = UID::default();
	loop {
		let new_card = mfrc522.picc_is_new_card_present();
		if let Some(atqa) = new_card {
			println!("New card detected. ATQA: {:04x}", atqa.bits());

			let status = mfrc522.picc_select(&mut uid);
			println!("Select: {:?} {:?}", status, uid);
			if status.is_ok() {
				println!("Card UID: {:?} | Type: {:?}", uid.as_bytes(), uid.picc_type());

				if authenticate_and_read_block_zero(&mut mfrc522, &uid) {
					let halt_status = mfrc522.picc_hlta();
					println!("Halt: {:?}", halt_status);
				}

				mfrc522.pcd_stopcrypto1().expect("MFRC522: pcd_stopcrypto1() failed.");
			}

			uid.clear();
		}

		thread::sleep(Duration::from_millis(500));
	}
}

pub fn main() {
	env_logger::init().unwrap_or_else(|e| println!("Could not init env_logger: {:?}.", e));

	match example() {
		Ok(_)  => println!("Done."),
		Err(e) => println!("Error: {:?}", e),
	}
}
