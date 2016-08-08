# MFRC522

Rust library for MFRC522 card reader.  
This is (an incomplete) attempt to port C++ MFRC522 library by COOQROBOT et. al.

This is still Work in Progress. API is highly unstable at the moment.  
Latest version of source code can be found @ https://github.com/emkw/rust-MFRC522.git

Any contributions are welcome.

## Status 

Reading 4B Card UID works.

REQA / WUPA - OK.
SELECT - Cascade (7B, 10B UID) is probably broken, No working card to test it.

i2cdev bus untested, there's a chance it may work.

## How to use it?

### Cargo.toml:
```toml
[dependencies.mfrc522]
git = "https://github.com/emkw/rust-MFRC522.git"
features = ["spidev", "self_test"]
```

### Cargo.toml features:
- `use_std` - compile without `no_std`.


- `host_crc` - calculate CRC_A on host rather than on PCD.
- `self_test` - provide MFRC522 self-test routine.


- `i2cdev` - provide i2cdev communication. This conflicts with `spidev`.
- `spidev` - provide spidev communication. This conflicts with `i2cdev`.

### Code:
Please see code in `examples.feat` directory.

## License

[MPL-2.0](https://www.mozilla.org/media/MPL/2.0/index.txt)

## Hardware info.

(This section is taken from `MFRC522.h`)

There are three hardware components involved:
1. The controller: eg. a microcontroller or microcomputer, capable of connecting to card reader.
2. The PCD (short for Proximity Coupling Device): NXP MFRC522 Contactless Reader IC.
3. The PICC (short for Proximity Integrated Circuit Card): A card or tag using the ISO 14443A interface, eg Mifare or NTAG203.

The controller and card reader uses an interface implemented in `MFRC522Bus` for communication. On MFRC522 hardware level it's  SPI, I2C or UART,
but it can be abstracted further eg. UART over TCP.  
The protocol is described in the MFRC522 datasheet: http://www.nxp.com/documents/data_sheet/MFRC522.pdf

The card reader and the tags communicate using a 13.56MHz electromagnetic field.
The protocol is defined in ISO/IEC 14443-3 Identification cards -- Contactless integrated circuit cards -- Proximity cards -- Part 3: Initialization and anticollision".
A free version of the final draft can be found at http://wg8.de/wg8n1496_17n3613_Ballot_FCD14443-3.pdf
Details are found in chapter 6, Type A – Initialization and anticollision.

If only the PICC UID is wanted, the above documents has all the needed information.
To read and write from MIFARE PICCs, the MIFARE protocol is used after the PICC has been selected.
  
The MIFARE Classic chips and protocol is described in the datasheets:
- 1K:   http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf
- 4K:   http://datasheet.octopart.com/MF1S7035DA4,118-NXP-Semiconductors-datasheet-11046188.pdf
- Mini: http://www.idcardmarket.com/download/mifare_S20_datasheet.pdf

The MIFARE Ultralight chip and protocol is described in the datasheets:
- Ultralight:   http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf
- Ultralight C: http://www.nxp.com/documents/short_data_sheet/MF0ICU2_SDS.pdf

### MIFARE Classic 1K (MF1S503x)
- Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
- The blocks are numbered 0-63.
- Block 3 in each sector is the Sector Trailer. See http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf sections 8.6 and 8.7:
  - Bytes 0-5:   Key A
  - Bytes 6-8:   Access Bits
  - Bytes 9:     User data
  - Bytes 10-15: Key B (or user data)
- Block 0 is read-only manufacturer data.
- To access a block, an authentication using a key from the block's sector must be performed first.
- Example: To read from block 10, first authenticate using a key from sector 3 (blocks 8-11).
- All keys are set to FFFFFFFFFFFFh at chip delivery.
- **Warning**: Please read section 8.7 "Memory Access". It includes this text: if the PICC detects a format violation the whole sector is irreversibly blocked.
- To use a block in "value block" mode (for Increment/Decrement operations) you need to change the sector trailer. Use PICC_SetAccessBits() to calculate the bit patterns.

### MIFARE Classic 4K (MF1S703x)
- Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
- The blocks are numbered 0-255.
- The last block in each sector is the Sector Trailer like above.

### MIFARE Classic Mini (MF1 IC S20)
- Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
- The blocks are numbered 0-19.
- The last block in each sector is the Sector Trailer like above.

### MIFARE Ultralight (MF0ICU1)
- Has 16 pages of 4 bytes = 64 bytes.
- Pages 0 + 1 is used for the 7-byte UID.
- Page 2 contains the last check digit for the UID, one byte manufacturer internal data, and the lock bytes (see http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf section 8.5.2)
- Page 3 is OTP, One Time Programmable bits. Once set to 1 they cannot revert to 0.
- Pages 4-15 are read/write unless blocked by the lock bytes in page 2.

### MIFARE Ultralight C (MF0ICU2)
- Has 48 pages of 4 bytes = 192 bytes.
- Pages 0 + 1 is used for the 7-byte UID.
- Page 2 contains the last check digit for the UID, one byte manufacturer internal data, and the lock bytes (see http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf section 8.5.2)
- Page 3 is OTP, One Time Programmable bits. Once set to 1 they cannot revert to 0.
- Pages 4-39 are read/write unless blocked by the lock bytes in page 2.
- Page 40 Lock bytes.
- Page 41 16 bit one way counter.
- Pages 42-43 Authentication configuration.
- Pages 44-47 Authentication key.

## Credits
Original C++ MFRC522 library:
- Based on code Dr.Leong   (WWW.B2CQSHOP.COM)
- Created by Miguel Balboa (circuitito.com), Jan, 2012.
- Rewritten by Søren Thing Andersen (access.thing.dk), fall of 2013 (Translation to English, refactored, comments, anti collision, cascade levels.)
- Extended by Tom Clement with functionality to write to sector 0 of UID changeable Mifare cards.
