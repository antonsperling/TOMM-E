#ifndef _NewAudioStreamer_h
#define _NewAudioStreamer_h

#include "ELClientSocket.h"

#include <SD.h>
#include <SPI.h>
// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 53; // (Or any suitable pin!)

File myFile;
File tempfile;
char bufFile[] = "00000000.wav";
char sdfile[] = "00000000.txt";
char filename[] = "00000000.txt";
unsigned char* lastFile;
boolean hascard = false;

boolean hasdata , written, aready, writeit;
unsigned long starttime , endtime, filesize;
float frequency;
float period , interval;
int const adport = 1; // Set the port to be used for input!
String temp;

// buf is used to store icoming data and is written to file when full
// 512 bytes is optimized for sdcard
#define BUF_SIZE 512
uint8_t bufa[BUF_SIZE];
uint8_t bufb[BUF_SIZE];
uint16_t bufcount;
unsigned long readings = 24000; //  initial sample size- kept small to avoid delay- enough to create data to look at
int i;
unsigned long grab, counter;
int countFiles = 0;

// Defines for clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
// Defines for setting register bits
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// alter prescalar here 32, 64, 128
// 32 is the fastest sampling rate
byte prescalar = 64;
byte wavheader[44];

void sendAudioData(byte* fileName, ELClientSocket * tcp) {
  Console.println(F("TcpVoiceStreamingClient: Sending data: "));
  myFile = SD.open(fileName, FILE_READ);
  if (!myFile) {
    Console.println(F("Could not open File for tcp transfer"));
  }
  uint8_t filesize = myFile.size();
  
  myFile.seek(0);
  tcp->sendStream(myFile);
  myFile.close();

}

void startad() {
  // Setup continuous reading of the adc port 'adport' using an interrupt

  cli();//disable interrupts
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  ADMUX |= adport;   //set up continuous sampling of analog pin adport
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  // pre scalar to set interrupt frequency:
  //ADCSRA |= (1 << ADPS2); // 16 prescalar 72Khz, but what could you do at that rate?
  // 128 prescalar - 9.4 Khz sampling
  if (prescalar == 128) ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  //32 prescaler - 16mHz/32=500kHz - produces 37 Khz sampling
  if (prescalar == 32) ADCSRA |= (1 << ADPS2) | (1 << ADPS0);
  // 64 prescalar produces 19.2 Khz sampling
  if (prescalar == 64) ADCSRA |= (1 << ADPS2) | (1 << ADPS1);

  ADCSRA |= (1 << ADATE); //enable auto trigger
  ADCSRA |= (1 << ADIE);
  //ADCSRA |= (1 << ADEN); //enable ADC- works fine, but so does the next line
  sbi(ADCSRA, ADEN); // //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements on interrupt
  writeit = false;
  starttime = millis();
  sei();//enable interrupts
}

void headmod(long value, byte location) {
  // write four bytes for a long
  tempfile.seek(location); // find the location in the file
  byte tbuf[4];
  tbuf[0] = value & 0xFF; // lo byte
  tbuf[1] = (value >> 8) & 0xFF;
  tbuf[2] = (value >> 16) & 0xFF;
  tbuf[3] = (value >> 24) & 0xFF; // hi byte
  tempfile.write(tbuf, 4); // write the 4 byte buffer
}

boolean fileopen() {
  // Assemble text output filename from grab variable

  countFiles++;

  if (countFiles > 99) {
    countFiles = 1;
  }

  bufFile[6] = countFiles / 10 + '0';
  bufFile[7] = countFiles % 10 + '0';

  if (SD.exists(bufFile)) {
    SD.remove(bufFile);
  }
  tempfile = SD.open(bufFile, FILE_WRITE | O_TRUNC);
  if (!tempfile) {
    Serial.println(F("Tempfile failed to open"));
    return (false);
  } else {
    tempfile.write(wavheader, 44); // write wav header
    tempfile.seek(44); //set data start
    hasdata = false;
    written = false;
    counter = 0;
    bufcount = 0;
    return (true);
  }
}

// Interrupt routine *******************************************
// this is the key to the program!!
ISR(ADC_vect) {
  if (counter < readings) {
    if (aready) { // get the new value from analogue port
      bufa[bufcount] = ADCH;
    } else {
      bufb[bufcount] = ADCH;
    }
    counter++; // increment data counter
    bufcount++; // increment buffer counter
    if (bufcount == BUF_SIZE) {
      if (writeit == false) {
        bufcount = 0;
        aready = ! aready;
        writeit = true; // flag that a write is needed
      } else { // wait for file write to complete
        bufcount--;
        counter--;
      }
    }
  } else {
    // All data collected

    cli();//disable interrupts
    cbi(ADCSRA, ADEN); // disable ADC
    sei();//enable interrupts
    endtime = millis();
    // write the last block
    if (aready) {
      tempfile.write(bufa, BUF_SIZE); // write the data block
    } else {
      // initiate block write from B
      tempfile.write(bufb, BUF_SIZE); // write the data block
    }
    Serial.println(F("Audio Buffering stopped"));
    period = endtime - starttime;
    frequency = float(readings) / period;
    interval = 1000 / frequency;
    tempfile.flush();
    // update wav header
    long datacount = readings;
    long setf = long((frequency * 1000) + 0.55555555);
    headmod(datacount + 36, 4); //set size of data +44-8
    headmod(setf, 24); //set sample rate Hz
    headmod(setf, 28); //set sample rate Hz
    headmod(datacount, 40); // set data size
    tempfile.close();
    hasdata = true; // flag the data presence
  }
}
// End Interupt section *******************************************

void startAudioRecording() {

  //Specimem frequency used for first reading to estimate initial sample duration
  if (prescalar == 32) frequency = 38.3829;
  if (prescalar == 64) frequency = 19.2300;
  if (prescalar == 128) frequency = 9.6182;

  // wavheader setup
  // little endian (lowest byte 1st)
  wavheader[0] = 'R';
  wavheader[1] = 'I';
  wavheader[2] = 'F';
  wavheader[3] = 'F';
  //wavheader[4] to wavheader[7] size of data + header -8
  wavheader[8] = 'W';
  wavheader[9] = 'A';
  wavheader[10] = 'V';
  wavheader[11] = 'E';
  wavheader[12] = 'f';
  wavheader[13] = 'm';
  wavheader[14] = 't';
  wavheader[15] = ' ';
  wavheader[16] = 16;
  wavheader[17] = 0;
  wavheader[18] = 0;
  wavheader[19] = 0;
  wavheader[20] = 1;
  wavheader[21] = 0;
  wavheader[22] = 1;
  wavheader[23] = 0;
  // wavheader[24] to wavheader[27] samplerate hz
  // wavheader[28] to wavheader[31] samplerate*1*1
  // optional bytes can be added here
  wavheader[32] = 1;
  wavheader[33] = 0;
  wavheader[34] = 8;
  wavheader[35] = 0;
  wavheader[36] = 'd';
  wavheader[37] = 'a';
  wavheader[38] = 't';
  wavheader[39] = 'a';
  //wavheader[40] to wavheader[43] sample number

  if (SD.begin(chipSelect)) {
    hascard = true;
    aready = true;
    card.init(SPI_FULL_SPEED, chipSelect);

    hascard = fileopen();
  }
  if (hascard == false) {
    Console.println(F("SD Problem"));
  }
  cli();
  startad();


  Console.println(F("Audio recording started"));
}

void processAudioRecording(ELClientSocket * tcp) {
  if (writeit) {
    if (aready) {
      tempfile.write(bufb, BUF_SIZE); // write the data block
    } else {
      // initiate block write from buf A
      tempfile.write(bufa, BUF_SIZE); // write the data block
    }
    writeit = false; // the write is done!
  }

  if (hasdata) {
    Console.println(F("Should publish audio Stream now"));
    sendAudioData(bufFile, tcp);

    hasdata = false;
    counter = 0;
    hascard = fileopen();
    if (!hascard) Console.println(F("SD Problem"));
    startad();

  }
}

#endif
