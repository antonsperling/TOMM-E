#ifndef _AudioStreamer_h
#define _AudioStreamer_h


#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define AUDIO_SAMPLE_RATE 16000 // Which sampleRate to use


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


//#################################################################
// Audio Sampling on ESP32
//#################################################################
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#define ADC_SAMPLES_COUNT 1000
// ADC1_CHANNEL_0 is GPIO 36 on ESP32 (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#_CPPv414adc1_channel_t)
#define ADC1_CHANNEL_0 0
int16_t abuf[ADC_SAMPLES_COUNT];
int16_t abufPos = 0;
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t adcTaskHandle;
//hw_timer_t * adcTimer = NULL; // our timer


int IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  abuf[abufPos++] = local_adc1_read(ADC1_CHANNEL_0);
  
  if (abufPos >= ADC_SAMPLES_COUNT) { 
    abufPos = 0;

    // Notify adcTask that the buffer is full.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(adcTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

/*********************************************************/

void startRecording(char *fileName, uint32_t sampleRate) {

  Serial.print(F("Start Recording: "));
  Serial.println(fileName);

  if (SD.exists(fileName)) {
    Serial.println(F("Deleted file, because already existed"));
    SD.remove(fileName);
  }

  //audio.startRecording(fileName, sampleRate, ANALOG_INPIN);
  // Do own ADC sample recording
  // #################################################################################################
  
  audioInitialized = true;
  startRec = millis();
}

/*********************************************************/

void stopRecording(char *fileName) {

  //audio.stopRecording(fileName);
  // Do own ADC sample recording
  // #################################################################################################
  Serial.println(F("Recording Stopped"));

/*********************************************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/
/** IMPLEMENT SEND OVER WIFI HERE ************************/


  File lastRec = SD.open(fileName);
  if (!lastRec) {
    Serial.println(F("Could not open File for tcp transfer"));
  }
  //uint8_t filesize = myFile.size();
  
  lastRec.seek(0);
  Serial.print(F("Going to send file over tcp. Filesize is "));
  uint32_t filesize = lastRec.size();
  Serial.println(filesize);
  lastRec.close();
  Serial.println(F("Finished send file over tcp"));
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
  Serial.println(F("Starting connection to server..."));
  if (wifiFtpCommandClient.connect(FTP_SERVER, FTP_COMMAND_PORT)) {
    Serial.println(F("connected to server"));

  }

  audioInitialized = true;
}

/*********************************************************/

void processAudioRecording() {
  if (!audioInitialized) {
    Serial.println(F("Audio not initialized. Not going to process anything..."));
    return;
  }

  if (millis() - startRec >= 4000) {
    stopRecording(newWavFile);
    startRec = millis();
  }
}

#endif
