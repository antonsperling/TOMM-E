#ifndef _AudioStreamer_h
#define _AudioStreamer_h


#include <SPI.h>
#include <SD.h>
#include <TMRpcm.h>
#include <WiFiEspAT.h>

#define SD_ChipSelectPin 53  //example uses hardware SS pin 53 on Mega2560
//#define SD_ChipSelectPin 4  //using digital pin 4 on arduino nano 328, can use other pins
#define ANALOG_INPIN A0 // Which analog pin to record from
#define AUDIO_SAMPLE_RATE 16000 // Which sampleRate to use

TMRpcm audio;   // create an object for use in this sketch

char newWavFile[] = "00000000.wav";

bool audioInitialized = false;
unsigned long startRec;
byte countFiles = 0;


/*********************************************************/

void startRecording(const char *fileName, uint32_t sampleRate) {

  Serial.print(F("Start Recording: "));
  Serial.println(fileName);

  if (SD.exists(fileName)) {
    Serial.println(F("Deleted file, because already existed"));
    SD.remove(fileName);
  }

  audio.startRecording(fileName, sampleRate, ANALOG_INPIN);
  audioInitialized = true;
  startRec = millis();
}

/*********************************************************/

void stopRecording(const char *fileName, WiFiClient * tcp) {

  audio.stopRecording(fileName);
  Serial.println(F("Recording Stopped"));

/*********************************************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/


  File lastRec = SD.open(fileName);
  if (!lastRec) {
    Console.println(F("Could not open File for tcp transfer"));
  }
  //uint8_t filesize = myFile.size();
  
  lastRec.seek(0);
  Console.print(F("Going to send file over tcp. Filesize is "));
  uint32_t filesize = lastRec.size();
  Console.println(filesize);
  lastRec.close();
  Console.println(F("Finished send file over tcp"));
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/*********************************************************/
  countFiles++;

  if (countFiles > 99) {
    countFiles = 0;
  }

  newWavFile[6] = countFiles / 10 + '0';
  newWavFile[7] = countFiles % 10 + '0';

  startRecording(newWavFile, AUDIO_SAMPLE_RATE);
  audioInitialized = true;
}

/*********************************************************/

void startAudioRecording() {
  startRecording(newWavFile, AUDIO_SAMPLE_RATE);
  Console.println(F("Starting connection to server..."));
  if (client.connect(TCP_SERVER, TCP_PORT)) {
    Console.println(F("connected to server"));

    //client.println("GET /asciilogo.txt HTTP/1.1");
    //client.print("Host: ");
    //client.println(server);
    //client.println("Connection: close");
    //client.println();
    //client.flush();
  }

  audioInitialized = true;
}

void processAudioRecording(WiFiClient * tcp) {
  if (!audioInitialized) {
    Console.println(F("Audio not initialized. Not going to process anything..."));
    return;
  }

  if (millis() - startRec >= 4000) {
    stopRecording(newWavFile, tcp);
    startRec = millis();
  }
}

#endif
