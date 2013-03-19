/*
ganked from http://www.arduino.cc/playground/Code/SDCARD
*/
/*
Basic instructions for recording data in an SD Card in native mode
The SD card (3.3 V) must be properly interfaced to Arduino (5 V)

The typical procedure is:
Initialize SPI
Initialize SD Card
START
-Blank vector of data (vBlock)
-Record data in vector of data
-Copy data from vector to CSD card
GOTO START

At your convenience:
-Copy data from CSD card to vector of data
-Read data from vector

Starting from there, you will have to build your own file system...

Useful links
http://elm-chan.org/docs/mmc/mmc_e.html
http://www.retroleum.co.uk/mmc_cards.html
http://www.maxim-ic.com/appnotes.cfm/an_pk/3969

No warranty, no claims, just fun

Didier Longueville invenit et fecit February 2010
*/

#ifdef SD

// Ports
int PIN_CS = PINB2;      // chip select
int PIN_MOSI = PINB3;    // master out slave in
int PIN_MISO = PINB4;    // master in slave out
int PIN_CLOCK = PINB5;   // clock

/********************** SPI SECTION BELOW **********************/

// SPI Variables
byte clr;     // dummy variable used to clear some of the SPI registers
byte spi_err; // SPI timeout flag, must be cleared manually

// send an SPI command, includes time out management
// returns spi_err: "0" is "no error"
byte spi_cmd(volatile char data) {
	spi_err = 0; // reset spi error
	SPDR = data; // start the transmission by loading the output byte into the spi data register
	int i = 0;
	while (!(SPSR & (1<<SPIF))) {
		i++;
		if (i >= 0xFF) {
			spi_err = 1;
			return(0x00);
		}
	}
	// returned value
	return(SPDR);
}

// initialize SPI port
void spi_initialize(void) {
	SPCR = (1<<SPE) | (1<<MSTR); // spi enabled, master mode
	clr = SPSR; // dummy read registers to clear previous results
	clr = SPDR;
}

/********************** SD CARD SECTION BELOW **********************/

// SD Card variables
#define blockSize 512          // block size (default 512 bytes)
byte vBlock[blockSize];        // set vector containing data that will be recorded on SD Card
byte vBuffer[16];

#define GO_IDLE_STATE 0x00     // resets the SD card
#define SEND_CSD 0x09          // sends card-specific data
#define SEND_CID 0x0A          // sends card identification
#define READ_SINGLE_BLOCK 0x11 // reads a block at byte address
#define WRITE_BLOCK 0x18       // writes a block at byte address
#define SEND_OP_COND 0x29      // starts card initialization
#define APP_CMD 0x37           // prefix for application command


// Send a SD command, num is the actual index, NOT OR'ed with 0x40.
// arg is all four bytes of the argument
byte sdc_cmd(byte commandIndex, long arg) {
	PORTB &= ~(1<<PIN_CS);   // assert chip select for the card
	spi_cmd(0xFF);           // dummy byte
	commandIndex |= 0x40;    // command token OR'ed with 0x40
	spi_cmd(commandIndex);   // send command
	for (int i=3; i>=0; i--) {
		spi_cmd(arg>>(i*8));   // send argument in little endian form (MSB first)
	}
	spi_cmd(0x95);           // checksum valid for GO_IDLE_STATE, not needed thereafter, so we can hardcode this value
	spi_cmd(0xFF);           // dummy byte gives card time to process
	byte res = spi_cmd(0xFF);
	return (res);  // query return value from card
}

// initialize SD card
// retuns 1 if successful
byte sdc_initialize(void) {
	// set slow clock: 1/128 base frequency (125Khz in this case)
	SPCR |=  (1<<SPR1) | (1<<SPR0); // set slow clock: 1/128 base frequency (125Khz in this case)
	SPSR &= ~(1<<SPI2X);            // No doubled clock frequency
	// wake up SD card
	PORTB |=  (1<<PIN_CS);          // deasserts card for warmup
	PORTB |=  (1<<PIN_MOSI);        // set MOSI high
	for(byte i=0; i<10; i++) {
		spi_cmd(0xFF);                // send 10 times 8 pulses for a warmup (74 minimum)
	}
	// set idle mode
	byte retries=0;
	PORTB &= ~(1<<PIN_CS);          // assert chip select for the card
	while(sdc_cmd(GO_IDLE_STATE, 0) != 0x01) { // while SD card is not in iddle state
		retries++;
		if (retries >= 0xFF) {
			return(NULL); // timed out!
		}
		delay_ms(5);
	}
	// at this stage, the card is in idle mode and ready for start up
	retries = 0;
	sdc_cmd(APP_CMD, 0); // startup sequence for SD cards 55/41
	while (sdc_cmd(SEND_OP_COND, 0) != 0x00) {
		retries++;
		if (retries >= 0xFF) {
			return(NULL); // timed out!
		}
		sdc_cmd(APP_CMD, 0);
	}
	// set fast clock, 1/4 CPU clock frequency (4Mhz in this case)
	SPCR &= ~((1<<SPR1) | (1<<SPR0)); // Clock Frequency: f_OSC / 4
	SPSR |=  (1<<SPI2X);              // Doubled Clock Frequency: f_OSC / 2
	return (0x01); // returned value (success)
}

// clear block content
void sdc_clearVector(void) {
	for (int i=0; i<blockSize; i++) {
		vBlock[i] = 0;
	}
}

// get nbr of blocks on SD memory card from
long sdc_totalNbrBlocks(void) {
	sdc_readRegister(SEND_CSD);
	// compute size
	long C_Size = ((vBuffer[0x08] & 0xC0) >> 6) | ((vBuffer[0x07] & 0xFF) << 2) | ((vBuffer[0x06] & 0x03) << 10);
	long C_Mult = ((vBuffer[0x08] & 0x80) >> 7) | ((vBuffer[0x08] & 0x03) << 2);
	return ((C_Size+1) << (C_Mult+2));
}

// read SD card register content and store it in vBuffer
void sdc_readRegister(byte sentCommand) {
	byte retries=0x00;
	byte res=sdc_cmd(sentCommand, 0);
	while(res != 0x00) {
		delay_ms(1);
		retries++;
		if (retries >= 0xFF) return; // timed out!
		res=spi_cmd(0xFF); // retry
	}
	// wait for data token
	while (spi_cmd(0xFF) != 0xFE);
	// read data
	for (int i=0; i<16; i++) {
		vBuffer[i] = spi_cmd(0xFF);
	}
	// read CRC (lost results in blue sky)
	spi_cmd(0xFF); // LSB
	spi_cmd(0xFF); // MSB
}

// write block on SD card
// addr is the address in bytes (multiples of block size)
void sdc_writeBlock(long blockIndex) {
	byte retries=0;
	while(sdc_cmd(WRITE_BLOCK, blockIndex * blockSize) != 0x00) {
		delay_ms(1);
		retries++;
		if (retries >= 0xFF) return; // timed out!
	}
	spi_cmd(0xFF); // dummy byte (at least one)
	// send data packet (includes data token, data block and CRC)
	// data token
	spi_cmd(0xFE);
	// copy block data
	for (int i=0; i<blockSize; i++) {
		spi_cmd(vBlock[i]);
	}
	// write CRC (lost results in blue sky)
	spi_cmd(0xFF); // LSB
	spi_cmd(0xFF); // MSB
	// wait until write is finished
	while (spi_cmd(0xFF) != 0xFF) delay_ms(1); // kind of NOP
}

// read block on SD card and copy data in block vector
// retuns 1 if successful
void sdc_readBlock(long blockIndex) {
	byte retries = 0x00;
	byte res = sdc_cmd(READ_SINGLE_BLOCK,  (blockIndex * blockSize));
	while(res != 0x00) {
		delay_ms(1);
		retries++;
		if (retries >= 0xFF) return; // timed out!
		res=spi_cmd(0xFF); // retry
	}
	// read data packet (includes data token, data block and CRC)
	// read data token
	while (spi_cmd(0xFF) != 0xFE);
	// read data block
	for (int i=0; i<blockSize; i++) {
		vBlock[i] = spi_cmd(0xFF); // read data
	}
	// read CRC (lost results in blue sky)
	spi_cmd(0xFF); // LSB
	spi_cmd(0xFF); // MSB
}

// // print vector of data
// void sdc_printVectorContent(void) {
// 	for (int i=0; i<blockSize; i++) {
// 		Serial.print("0x");
// 		if (vBlock[i] <= 0x0F) Serial.print("0");
// 		Serial.print(vBlock[i], HEX);
// 		Serial.print(" ");
// 		// append crlf to each line of 16 bytes
// 		if (((i+1) % 16) == 0) Serial.println();
// 	}
// 	Serial.println();
// }
// 
// /********************** MAIN ROUTINES SECTION  BELOW **********************/
// 
// void setup() {
// 	// Set ports
// 	// Data in
// 	DDRB &= ~(1<<PIN_MISO);
// 	// Data out
// 	DDRB |=  (1<<PIN_CLOCK);
// 	DDRB |=  (1<<PIN_CS);
// 	DDRB |=  (1<<PIN_MOSI);
// 	// Initialize serial communication
// 	Serial.begin(115200);
// 	// Initialize SPI and SDC
// 	spi_err=0;        // reset SPI error
// 	spi_initialize(); // initialize SPI port
// 	sdc_initialize(); // Initialize SD Card
// 	Serial.print(sdc_totalNbrBlocks(), DEC);
// 	Serial.println(" blocks");
// }
// 
// void loop() {
// 	// This is just an example
// 	Serial.println("Writing blocks...");
// 	for (int b=0; b<255; b++) {
// 		Serial.print("Writing block ");
// 		Serial.println(b, HEX);
// 		for (int i=0; i<blockSize; i++){
// 			vBlock[i] = b; // write incremental data
// 		}
// 		sdc_writeBlock(b);         // copy vector of data on SD card
// 	}
// 	Serial.println("Reading blocks...");
// 	for (int b=0; b<255; b++) {
// 		Serial.print("Reading block ");
// 		Serial.println(b, HEX);
// 		sdc_readBlock(b);          // copy SD card block of data in vector of data
// 		sdc_printVectorContent();  // print vector of data content
// 	}
// }

#endif /* SD */
