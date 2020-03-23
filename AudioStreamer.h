#ifndef _AudioStreamer_h
#define _AudioStreamer_h


#include <SPI.h>
#include <SD.h>
#include <TMRpcm.h>
#include <WiFiEspAT.h>
#include <PubSubClient.h>

#define SD_ChipSelectPin 53  //example uses hardware SS pin 53 on Mega2560
//#define SD_ChipSelectPin 4  //using digital pin 4 on arduino nano 328, can use other pins
#define ANALOG_INPIN A0 // Which analog pin to record from
#define AUDIO_SAMPLE_RATE 16000 // Which sampleRate to use

#define ESP8266Serial Serial2
#define ESP8266_BAUDRATE    115200      // baudrate used for communication with esp8266 Wifi module

TMRpcm audio;   // create an object for use in this sketch

char newWavFile[] = "00000000.wav";

bool audioInitialized = false;
unsigned long startRec;
uint8_t countFiles = 0;

WiFiClient wifiFtpCommandClient;
WiFiClient wifiFtpDataClient;
WiFiClient wifiMqttClient;
PubSubClient mqttClient(wifiMqttClient);

boolean mqttconnected = false;
unsigned long voiceActivatedTime = 0L;

// IP address for this demo is a local IP.
// Replace it with the IP address where you have a FTP/MQTT server running
char * const FTP_SERVER PROGMEM = "192.168.178.61";
char * const MQTT_SERVER PROGMEM = "192.168.178.61";
uint16_t const FTP_COMMAND_PORT PROGMEM = 21;
uint16_t const MQTT_PORT PROGMEM = 1883;



/*********************************************************/

void startRecording(const char *fileName, uint32_t sampleRate) {

  Console.print(F("Start Recording: "));
  Console.println(fileName);

  if (SD.exists(fileName)) {
    Console.println(F("Deleted file, because already existed"));
    SD.remove(fileName);
  }

  audio.startRecording(fileName, sampleRate, ANALOG_INPIN);
  audioInitialized = true;
  startRec = millis();
}

/*********************************************************/

void stopRecording(const char *fileName) {

  audio.stopRecording(fileName);
  Console.println(F("Recording Stopped"));

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
}

/*********************************************************/
// startAudioRecording is beeing invoked only once, when
// switching robot state from STATE_INIT
void startAudioRecording() {
  startRecording(newWavFile, AUDIO_SAMPLE_RATE);
  Console.println(F("Starting connection to server..."));
  if (wifiFtpCommandClient.connect(FTP_SERVER, FTP_COMMAND_PORT)) {
    Console.println(F("connected to server"));

  }

  audioInitialized = true;
}

/*********************************************************/

void processAudioRecording() {
  if (!audioInitialized) {
    Console.println(F("Audio not initialized. Not going to process anything..."));
    return;
  }

  if (millis() - startRec >= 4000) {
    stopRecording(newWavFile);
    startRec = millis();
  }
}

#endif
