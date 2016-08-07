// This example needs features: i2cdev
extern crate i2cdev;
extern crate mfrc522;

use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use mfrc522::MFRC522;
use mfrc522::pcd::reg::Reg;

fn example() -> Result<(), LinuxI2CError> {
	let mut bus = try!(LinuxI2CDevice::new("/dev/i2c-1", 0));
	let mut mfrc522 = MFRC522::init(&mut bus);

	let version = mfrc522.register_read(Reg::Version);
	println!("Version: {:X}", version);

	Ok(())
}

fn main() {
	match example() {
		Ok(_)  => println!("Done."),
		Err(e) => println!("Error: {:?}", e),
	}
}
