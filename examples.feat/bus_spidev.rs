/// This example needs features: spidev
extern crate spidev;
extern crate mfrc522;

use std::io;
use std::path::Path;

use spidev::{Spidev, SpidevOptions,SPI_MODE_0};
use mfrc522::MFRC522;
use mfrc522::pcd::reg::Reg;

pub fn spi_open<P: AsRef<Path>>(path: P) -> io::Result<Spidev> {
	let mut options = SpidevOptions::new();
	options.bits_per_word(8)
		.lsb_first(false)
		.mode(SPI_MODE_0)
		.max_speed_hz(4_000_000);
	let mut spi = try!(Spidev::open(path.as_ref()));
	try!(spi.configure(&options));

	Ok(spi)
}

#[allow(dead_code)]
fn example() -> io::Result<()> {
	let mut bus = try!(spi_open("/dev/spidev0.0"));
	let mut mfrc522 = MFRC522::init(&mut bus);
	let version = mfrc522.register_read(Reg::Version);
	println!("Version: {:X}", version);

	Ok(())
}

#[allow(dead_code)]
fn main() {
	match example() {
		Ok(_)  => println!("Done."),
		Err(e) => println!("Error: {:?}", e),
	}
}
