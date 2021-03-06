/// This example needs features: spidev self_test
extern crate spidev;
extern crate mfrc522;

extern crate env_logger;

mod bus_spidev;
use bus_spidev::spi_open;

use std::io;
use std::thread;
use std::time::Duration;

use mfrc522::MFRC522;
use mfrc522::pcd::Reg;
use mfrc522::picc;

fn example() -> io::Result<()> {
	let mut bus = try!(spi_open("/dev/spidev0.0"));
	let mut mfrc522 = MFRC522::init(&mut bus).expect("MFRC522 Initialization failed");

	let version = mfrc522.register_read(Reg::Version).expect("Could not read version");
	println!("Version: {:X}.", version);

	println!("Starting self-test.");
	let test_status = mfrc522.self_test();
	if test_status.is_ok() {
		println!("Self-test: PASSED.");
	} else {
		println!("Self-test: FAILED: {:?}", test_status);
	}

	// Re-init after self-test.
	mfrc522.pcd_soft_reset().expect("Failed to reset MFRC522");
	mfrc522.pcd_init().expect("Failed to re-init MFRC522");

	let mut uid = picc::UID::default();
	loop {
		let new_card = mfrc522.picc_is_new_card_present();
		if let Some(atqa) = new_card {
			println!("New card detected. ATQA: {:04x}", atqa.bits());

			let status = mfrc522.picc_select(&mut uid);
			println!("Select: {:?} {:?}", status, uid);
			if status.is_ok() {
				println!("Card UID: {:?} | Type: {:?}", uid.as_bytes(), uid.picc_type());

				let halt_status = mfrc522.picc_hlta();
				println!("Halt: {:?}", halt_status);
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
