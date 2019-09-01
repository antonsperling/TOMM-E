#include "Robot.h"

// Trigger and Echo Pins used on UNO for the UltraSonic Sensors.
// I am using the same Pin for Trigger and Echo to not waste Pins on the UNO.
// That means, that both Pins (Trigger,Echo) on the UltraSonic Sensor are attached to the same digital Pin.
// Search for Using UltraSonic Sensor in 3-Pin Mode for more Information.
#define TRIG_PIN_LEFT  4
#define ECHO_PIN_LEFT  4
#define TRIG_PIN_RIGHT  2
#define ECHO_PIN_RIGHT  2

#define MAX_DISTANCE 150 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define DELAY_BETWEEN_PINGS 50 // Wait at least 50ms between pings
#define DISTANCE_BARRIER 50 // If barrier/obstacle is closer than 50cm, try to turn left/right

#define SERVO_PIN_LEFT 3 // pin number on UNO for left Servo. Must be one of the six PWM Pins!
#define SERVO_PIN_RIGHT 5 // pin number on UNO for right Servor. Must be one of the six PWM Pins!
#define SERVO_PIN_TURN_HEAD 6 // pin number on UNO for Servo that turns Head. Must be one of the six PWM Pins!
#define SERVO_PIN_LIFT_HEAD 9 // pin number on UNO for Servo that lifts Head. Must be one of the six PWM Pins!
#define SERVO_PIN_ARM_LEFT 10 // pin number on UNO for Servo that turns Head. Must be one of the six PWM Pins!
#define SERVO_PIN_ARM_RIGHT 11 // pin number on UNO for Servo that lifts Head. Must be one of the six PWM Pins!

// after making the servos continuos rotation, the center angle has to be identified
// via manual probe. If everything went okay, it should be near to 90 degree. Mine are 92 and 93.
// Change this for your servos!
#define CENTER_ANGLE_LEFT 92
#define CENTER_ANGLE_RIGHT 93

#define FORW_LEFT 180
#define FORW_RIGHT 0
#define BACKW_LEFT 0
#define BACKW_RIGHT 180

#define LOOK_RIGHT 130
#define LOOK_LEFT 50
#define LOOK_CENTER 89
#define LIFT_LOOK_UP 80
#define LIFT_LOOK_CENTER 140

#define LEFT_ARM_UP 70
#define LEFT_ARM_DOWN 180
#define RIGHT_ARM_UP 110
#define RIGHT_ARM_DOWN 0

#define BATTERY_VOLTAGE_CHECK_PIN A0 // For battery voltage Check, the analog Pin 0 (A0) is beeing used.

#define STATE_INIT 0
#define STATE_ERROR 99 // not implemented yet
#define STATE_FORW 1
#define STATE_BACKW 2
#define STATE_TURN_LEFT 3
#define STATE_TURN_RIGHT 4
#define STATE_MP3_ERROR 5
#define STATE_STOPPED_TO_TALK 6
#define STATE_BAT_LOW 7 // voltage sensor not working at the moment, so state is not set
#define STATE_BAT_EMPTY 8 // not implemented yet
#define STATE_VOICE_ACTIVATED 9 // hotword detected
#define STATE_STUCK 98 // not implemented yet

int currentState = STATE_INIT;
int nextState = STATE_INIT;
long lastStateChange = 0UL; // when did last State change occur
#define MAX_STATE_TIME 30000 // only stay in one state for max. 30 seconds

Servo servoLeft;
Servo servoRight;
Servo servoLiftHead;
Servo servoTurnHead;
Servo servoArmLeft; // less degree == arm up
Servo servoArmRight; // less degree == arm down

NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

float durationLeft, distanceLeft, durationRight, distanceRight;
long loopCount = 0;
long nextPingTime = 0UL;
long nextCryForPower = 0UL;
long blockDisplayChangeUntil = 0UL;
int firstTriangleXPos = 0;

// Measure battery voltage using a voltage divider. Currently not working on my setup
float currentVoltage;
float R1 = 32000.0;
float R2 = 47000.0;

// DFPlayer RX pin is connected to UNO Pin 12 and TX pin to UNO Pin 13
#ifdef HAS_MP3
DFRobotDFPlayerMini mp3;
#endif

#ifdef HAS_DISPLAY
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED
#endif

#ifdef HAS_SNIPS
#include "AudioStreamer.h"
ELClient esp(&ESP8266Serial, &ESP8266Serial);
//ELClient esp2(&Serial2, &Serial2);
ELClientCmd cmd(&esp);
ELClientMqtt mqtt(&esp);
ELClientSocket tcp(&esp);

boolean mqttconnected = false;
unsigned long voiceActivatedTime = 0L;
// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins
void wifiCb(void* response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if (status == STATION_GOT_IP) {
      Console.println(F("WIFI CONNECTED"));
    } else {
      Console.print(F("WIFI NOT READY: "));
      Console.println(status);
    }
  }
}

// Callback when MQTT is connected
void mqttConnectedCb(void* response) {
  Console.println(F("MQTT connected!"));
  mqttconnected = true;
}

// MqttClient::Callback when MQTT is disconnected
void mqttDisconnectedCb(void* response) {
  Console.println(F("MQTT disconnected"));
  mqttconnected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttDataCb(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Console.print(F("MQTT Received: topic="));
  String topic = res->popString();
  Console.println(topic);

  Console.print(F("data="));
  String data = res->popString();
  Console.println(data);

  if (topic == "hermes/hotword/default/detected") {
    setNextState(STATE_VOICE_ACTIVATED);
  }
}

void mqttPublishedCb(void* response) {
  Console.println(F("MQTT published"));
}

// Callback for TCP socket, called if data was sent or received
// Receives socket client number, can be reused for all initialized TCP socket connections
void tcpCb(uint8_t resp_type, uint8_t client_num, uint16_t len, char *data) {
  Console.println("tcpCb connection #" + String(client_num));
  if (resp_type == USERCB_SENT) {
    Console.println("\tSent " + String(len) + " bytes over client#" + String(client_num));
  } else if (resp_type == USERCB_RECV) {
    char recvData[len + 1]; // Prepare buffer for the received data
    memcpy(recvData, data, len); // Copy received data into the buffer
    recvData[len] = '\0'; // Terminate the buffer with 0 for proper printout!

    Console.println("\tReceived " + String(len) + " bytes over the client on connection #" + String(client_num));
    Console.println("\tReceived: " + String(recvData));
  } else if (resp_type == USERCB_RECO) {
    if (len != -11) { // ignore "not connected" error, handled in USERCB_CONN
      Console.print("\tConnection problem: ");
      Console.println(len);
    }
  } else if (resp_type == USERCB_CONN) {
    if (len == 0) {
      Console.println("\tDisconnected");
    } else {
      Console.println("\tConnected");
    }
  } else {
    Console.println("Received invalid response type");
  }
}
#endif

void setup() {
  // My MAX9814 Microphone PCB deliveres the output at 1.25V DC Offset.
  // That should deliver voltages between 0V and 2.5V coming into the analog pin A0 on Mega.
#ifdef HAS_SNIPS
  analogReference(INTERNAL2V56);
  pinMode(A0, INPUT);
#endif


  Console.begin(CONSOLE_BAUDRATE); // Open serial monitor at 9600 baud to see state changes an distances measured.

  Console.println("First invocation of console print");
  servoLeft.attach(SERVO_PIN_LEFT);
  servoRight.attach(SERVO_PIN_RIGHT);
  servoLiftHead.attach(SERVO_PIN_LIFT_HEAD);
  servoTurnHead.attach(SERVO_PIN_TURN_HEAD);
  servoArmLeft.attach(SERVO_PIN_ARM_LEFT);
  servoArmRight.attach(SERVO_PIN_ARM_RIGHT);

#ifdef HAS_DISPLAY
  u8g2.begin(); // Enable Display
  //u8g2.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
#endif

  fullStop();  // set Servos to Center == Stop
  servoLiftHead.write(LIFT_LOOK_CENTER);
  //delay(3000); // wait for 3 seconds after power up, to not run away without closing backdoor ;-)

#ifdef HAS_MP3
  DFPlayerPort.begin(DFPLAYER_BAUDRATE);
  if (!mp3.begin(DFPlayerPort)) {  //If connection to DFPlayer Module not available, stop here
    Console.println(F("Connection to DFPlayer not possible. Check and Reset."));
    while (true); // loop forever
  }
  mp3.setTimeOut(500); //Set serial communictaion time out 500ms
  mp3.volume(10);  //Set volume value (0~30).
  mp3.outputDevice(DFPLAYER_DEVICE_SD);
#endif

#ifdef HAS_SNIPS
  ESP8266Serial.begin(ESP8266_BAUDRATE);
  Console.println(F("Going to initialize AudioStreamer"));
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
  //esp2.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
  delay(5000); //wait for esp8266 to initialize properly
  bool wifiok;
  do {
    wifiok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!wifiok) Console.println(F("EL-MQTT-Client sync failed!"));
  } while (!wifiok);
  Console.println(F("EL-MQTT-Client synced!"));

  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnectedCb);
  mqtt.disconnectedCb.attach(mqttDisconnectedCb);
  mqtt.publishedCb.attach(mqttPublishedCb);
  mqtt.dataCb.attach(mqttDataCb);
  mqtt.setup();
  Console.println(F("EL-MQTT ready"));

  // Set up the TCP socket client for a connection to <TCP_SERVER> on port <>, this doesn't connect to that server,
  // it just sets-up stuff on the esp-link side and waits until we send some data
  int tcpConnNum = tcp.begin(TCP_SERVER, TCP_PORT, SOCKET_TCP_CLIENT, tcpCb); // SOCKET_CLIENT ==> we don't expect a response
  //int tcpConnNum = 0;
  if (tcpConnNum < 0) {
    Console.println(F("TcpVoiceStreamingClient: TCP socket setup failed, try again in 10 seconds after reboot"));
    delay(10000);
    asm volatile ("  jmp 0");
  } else {
    Console.println("TcpVoiceStreamingClient: " + String(TCP_SERVER) + ":" + String(TCP_PORT) + " is served over connection number # = " + String(tcpConnNum));
  }

  audio.speakerPin = 45; //5,6,11 or 46 on Mega, 9 on Uno, Nano, etc
  pinMode(46, OUTPUT); //Pin pairs: 9,10 Mega: 5-2,6-7,11-12,46-45

  if (!SD.begin(SD_ChipSelectPin)) {
    Serial.println(F("Failed to init SD card"));
    return;
  } else {
    Serial.println(F("SD OK"));
  }
  // The audio library needs to know which CS pin to use for recording
  audio.CSPin = SD_ChipSelectPin;


#endif
  lastStateChange = millis();
  Console.println(F("setup() done"));
}

void loop() {

  readSensors();
  if (currentState == nextState
      && (lastStateChange + MAX_STATE_TIME) < millis()
      && currentState != STATE_ERROR
      && currentState != STATE_BAT_EMPTY) {
    Console.println(F("lastStateChange longer than MAX_STATE_TIME. Switching to State STOPPED_TO_TALK"));
    setNextState(STATE_STOPPED_TO_TALK);
  }
  if (loopCount % 100000 == 0) {
    Console.print(F("100.000 loops done. "));
    Console.print(F("Current State is "));
    Console.println(currentState);
    if (currentState == STATE_FORW && nextState == STATE_FORW) {
      setNextState(STATE_STOPPED_TO_TALK);
    }
  }
  currentState = nextState;

  switch (currentState) {
    case STATE_ERROR:
      Console.println(F("Ran into Error. Won't go further. Please reset System"));
      delay(60000);
      break;
    case STATE_FORW:
      //showDistanceOnDisplay();
      checkDistance();
      checkVoltage();
      checkMp3();
      break;
    case STATE_INIT:
      showDistanceOnDisplay();
      checkVoltage();
      checkMp3();
      playBackgroundSong();
      startAudioStream();
      subscribeToTopics();
      setNextState(STATE_FORW);
      break;
    case STATE_TURN_LEFT:
    case STATE_TURN_RIGHT:
      //showDistanceOnDisplay();
      checkDistance();
      break;
    case STATE_STOPPED_TO_TALK:
      lookUp();
      armsUp();
      playSomeMusic();
      delay(4000); // REMOVE ME !!!!!!!!!!!!!!!!!!!!!
      setNextState(STATE_FORW);
      break;
    case STATE_BAT_LOW:
      checkVoltage();
      lookForward();
      armsUp();
      cryForPower();
      delay(20000);
      break;
    case STATE_BAT_EMPTY:
      Console.println(F("Battery empty. Will go no further."));
      displayBatEmpty();
      delay(120000);
      break;
    case STATE_VOICE_ACTIVATED:
      waitForVoiceCommand();
      //displayWaitForCommand();
      //setNextState(STATE_FORW);
      break;
    default:
      break;

  }
  loopCount++;
}

void setNextState(int state) {
  if (state != currentState) {
    lastStateChange = millis();
    Console.print(F("Last state was "));
    Console.print(currentState);
    Console.print(F(". Will switch to State "));
    Console.println(state);
  }

  nextState = state;
  switch (nextState) {
    case STATE_FORW:
      displayLuckyEyes();
      lookForward();
      armsDown();
      goForward();
      break;
    case STATE_TURN_LEFT:
      drawTriangles(true);
      armsDown();
      //displayBigText("<<<<<<<");
      lookLeft();
      turnLeft();
      break;
    case STATE_TURN_RIGHT:
      drawTriangles(false);
      armsDown();
      //displayBigText(">>>>>>>");
      lookRight();
      turnRight();
      break;
    case STATE_BAT_LOW:
    case STATE_BAT_EMPTY:
      fullStop();
      break;
    case STATE_STOPPED_TO_TALK:
      //displayEyes();
      lookForward();
      fullStop();
      break;
    case STATE_VOICE_ACTIVATED:
      Console.println(F("Voice activation recognized"));
      voiceActivatedTime = millis();
      lookForward();
      fullStop();
      break;
    default:
      break;
  }

}

void readSensors() {
  pingDistance();
  readVoltage();
  readVoiceInput();
  readMQTT();
}

void pingDistance() {

  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  if (nextPingTime > millis()) {
    return;
  }

  durationLeft = sonarLeft.ping();
  distanceLeft = (durationLeft / 2) * 0.0343;

  durationRight = sonarRight.ping();
  distanceRight = (durationRight / 2) * 0.0343;
  if (distanceRight == 0) {
    distanceRight = 999;
  }
  if (distanceLeft == 0) {
    distanceLeft = 999;
  }
  nextPingTime = millis() + DELAY_BETWEEN_PINGS;
}

void goForward() {
#ifdef DONT_MOVE
  return;
#endif
  servoLeft.write(FORW_LEFT);
  servoRight.write(FORW_RIGHT);
}

void goBackward() {
#ifdef DONT_MOVE
  return;
#endif
  servoLeft.write(BACKW_LEFT);
  servoRight.write(BACKW_RIGHT);
}

void turnLeft() {
#ifdef DONT_MOVE
  return;
#endif
  servoLeft.write(BACKW_LEFT);
  servoRight.write(FORW_RIGHT);
}
void turnLeft30Degree() {
#ifdef DONT_MOVE
  return;
#endif
  fullStop();
  servoLeft.write(BACKW_LEFT);
  servoRight.write(FORW_RIGHT);
  delay(1000);
  fullStop();
}
void lookLeft() {
#ifdef DONT_MOVE
  return;
#endif
  if (servoTurnHead.read() != LOOK_LEFT) {
    servoTurnHead.write(LOOK_LEFT);
  }
  if (servoLiftHead.read() != LIFT_LOOK_CENTER) {
    servoLiftHead.write(LIFT_LOOK_CENTER);
  }
}
void lookRight() {
#ifdef DONT_MOVE
  return;
#endif
  if (servoTurnHead.read() != LOOK_RIGHT) {
    servoTurnHead.write(LOOK_RIGHT);
  }
  if (servoLiftHead.read() != LIFT_LOOK_CENTER) {
    servoLiftHead.write(LIFT_LOOK_CENTER);
  }
}
void lookForward() {
#ifdef DONT_MOVE
  return;
#endif
  if (servoTurnHead.read() != LOOK_CENTER) {
    servoTurnHead.write(LOOK_CENTER);
  }
  if (servoLiftHead.read() != LIFT_LOOK_CENTER) {
    servoLiftHead.write(LIFT_LOOK_CENTER);
  }
}
void lookUp() {
#ifdef DONT_MOVE
  return;
#endif
  if (servoTurnHead.read() != LOOK_CENTER) {
    servoTurnHead.write(LOOK_CENTER);
  }
  if (servoLiftHead.read() != LIFT_LOOK_UP) {
    servoLiftHead.write(LIFT_LOOK_UP);
  }
}

void armsUp() {
#ifdef DONT_MOVE
  return;
#endif
  if (servoArmLeft.read() != LEFT_ARM_UP) {
    servoArmLeft.write(LEFT_ARM_UP);
  }
  if (servoArmRight.read() != RIGHT_ARM_UP) {
    servoArmRight.write(RIGHT_ARM_UP);
  }
}

void armsDown() {
#ifdef DONT_MOVE
  return;
#endif
  if (servoArmLeft.read() != LEFT_ARM_DOWN) {
    servoArmLeft.write(LEFT_ARM_DOWN);
  }
  if (servoArmRight.read() != RIGHT_ARM_DOWN) {
    servoArmRight.write(RIGHT_ARM_DOWN);
  }
}

void turnRight() {
#ifdef DONT_MOVE
  return;
#endif
  servoLeft.write(FORW_LEFT);
  servoRight.write(BACKW_RIGHT);
}
void turnRight30Degree() {
#ifdef DONT_MOVE
  return;
#endif
  fullStop();
  servoLeft.write(FORW_LEFT);
  servoRight.write(BACKW_RIGHT);
  delay(1000);
  fullStop();
}

void fullStop() {
#ifdef DONT_MOVE
  return;
#endif
  servoLeft.write(CENTER_ANGLE_LEFT); // 92° == center
  servoRight.write(CENTER_ANGLE_RIGHT); // 93° == center
}

void checkDistance() {
  if (distanceLeft > DISTANCE_BARRIER && distanceRight > DISTANCE_BARRIER) {
    if (currentState != STATE_FORW) {
      Console.println(F("Distances clear. Will switch to State FORW."));
      setNextState(STATE_FORW);
    }
    return;
  }

  if ((distanceLeft < DISTANCE_BARRIER && distanceLeft > 0) || (distanceRight < DISTANCE_BARRIER && distanceRight > 0)) {
    int numPings = 5;

    int avgLeft = 0;
    int avgRight = 0;
    for (int thisReading = 0; thisReading < numPings; thisReading++) {
      pingDistance();
      delay(DELAY_BETWEEN_PINGS);
      avgLeft = avgLeft + distanceLeft;
      avgRight = avgRight + distanceRight;
    }
    avgLeft = avgLeft / numPings;
    avgRight = avgRight / numPings;

    Console.print(F("Measured average left  = "));
    Console.println(avgLeft);
    Console.print(F("Measured average right = "));
    Console.println(avgRight);

    if (avgLeft < avgRight && avgLeft < DISTANCE_BARRIER && avgLeft != 0) {
      Console.print(F("Distance left is "));
      Console.print(avgLeft);
      Console.println(F(". Will turn to right."));
      setNextState(STATE_TURN_RIGHT);
    }
    if (avgRight < avgLeft && avgRight < DISTANCE_BARRIER && avgRight != 0) {
      Console.print(F("Distance right is "));
      Console.print(avgLeft);
      Console.println(F(". Will turn to left."));
      setNextState(STATE_TURN_LEFT);
    }
  }
}

void readVoltage() {
  // This wont work with the audioStream over WiFi.
  // So disabled for now...
  //int value = analogRead(BATTERY_VOLTAGE_CHECK_PIN);
  //currentVoltage = value * (5.0/1024);//*((R1 + R2)/R2);
  if (currentVoltage == 0) {
    currentVoltage = 999;
  }
}

void startAudioStream() {
#ifdef HAS_SNIPS
  startAudioRecording();
#endif
}
void readVoiceInput() {
#ifdef HAS_SNIPS
  processAudioRecording(&tcp);
#endif
}

void readMQTT() {
#ifdef HAS_SNIPS
  esp.Process();
#endif
}

void waitForVoiceCommand() {
#ifdef HAS_SNIPS
  if (millis() - voiceActivatedTime > 7000) {
    Console.println(F("Waited > 7s for voice command."));
    setNextState(STATE_FORW);
  }
#endif
}

void subscribeToTopics() {
#ifdef HAS_SNIPS
  mqtt.subscribe(F("hermes/hotword/default/detected"));
  Console.println(F("Done Subscription to hotword topic"));
#endif
}
void checkVoltage() {
  /*
    if (currentVoltage < 4.0) {
    setNextState(STATE_BAT_LOW);
    Console.print(F("Voltage = "));
    Console.println(currentVoltage);
    return;
    }
    if (currentVoltage < 3.7) {
    setNextState(STATE_BAT_EMPTY);
    Console.print(F("Voltage = "));
    Console.println(currentVoltage);
    return;
    }
  */
}

// #####################################################################################
// #####################################################################################
// ##
// ## Following the secondary functions for display (SSD1306) and sound (DFPlayer)
// ## All these might be disabled and therefore unused
// ##
// #####################################################################################
// #####################################################################################

void checkMp3() {
#ifdef HAS_MP3
  if (mp3.available()) { // If DFPlayer Module is sending data, print messages on display and serial
    printMp3Detail(mp3.readType(), mp3.read());
  }
#endif
}

void playSomeMusic() {
#ifdef HAS_MP3
  if (mp3.readCurrentFileNumber() >= 0) {
    //if currently playing, dont interrupt
    return;
  }
  mp3.volume(20);  //Set volume value (0~30).
  Console.println(F("Started to play MP3 File No. 1"));
  mp3.playMp3Folder(1); // TODO: Some music
  Console.println(F("STOPPED playing MP3"));
#endif
}

void playBackgroundSong() {
#ifdef HAS_MP3
  if (mp3.readCurrentFileNumber() >= 0) {
    //if currently playing, dont interrupt
    return;
  }
  mp3.volume(10);  //Set volume value (0~30).
  mp3.playMp3Folder(3); // TODO: Some music
#endif
}
void cryForPower() {
  //showVoltageOnDisplay();
  displayBatLow();
  if (nextCryForPower > millis()) {
    return;
  }
#ifdef HAS_MP3
  mp3.playMp3Folder(2); // TODO: Sound for low power
#endif
  nextCryForPower = millis() + 60000UL;
}

void displayBatLow() {
#ifdef HAS_DISPLAY
  if (blockDisplayChangeUntil > millis()) {
    return;
  }
  u8g2.clearBuffer();
  u8g2.drawXBMP(31, 0, Bat_low_width, Bat_low_height, Bat_low_bits);
  u8g2.sendBuffer();
#endif
}
void displayBatEmpty() {
#ifdef HAS_DISPLAY
  if (blockDisplayChangeUntil > millis()) {
    return;
  }
  u8g2.clearBuffer();
  u8g2.drawXBMP(31, 0, Bat_empty_width, Bat_empty_height, Bat_empty_bits);
  u8g2.sendBuffer();
#endif
}
void displayLuckyEyes() {
#ifdef HAS_DISPLAY
  if (blockDisplayChangeUntil > millis()) {
    return;
  }
  u8g2.clearBuffer();
  u8g2.drawXBMP(11, 0, Lucky_width, Lucky_height, Lucky_bits);
  u8g2.sendBuffer();
  blockDisplayChangeUntil = millis() + 100UL;
#endif
}

void drawTriangles(boolean leftTurn) {
#ifdef HAS_DISPLAY
  if (blockDisplayChangeUntil > millis()) {
    return;
  }
  u8g2.clearBuffer();
  int x0 = firstTriangleXPos;
  int x1 = 0;
  if (!leftTurn) {
    firstTriangleXPos = firstTriangleXPos - 3;
    if (firstTriangleXPos < 0) {
      firstTriangleXPos = 24;
      x0 = firstTriangleXPos;
    }
    x1 = x0 - 16;
  } else {
    firstTriangleXPos = firstTriangleXPos + 3;
    if (firstTriangleXPos > 24) {
      firstTriangleXPos = 0;
      x0 = firstTriangleXPos;
    }
    x1 = x0 + 16;
  }
  u8g2.drawTriangle(x0, 0, x1, 16, x0, 31);
  u8g2.drawTriangle(x0 + 24, 0, x1 + 24, 16, x0 + 24, 31);
  u8g2.drawTriangle(x0 + 48, 0, x1 + 48, 16, x0 + 48, 31);
  u8g2.drawTriangle(x0 + 72, 0, x1 + 72, 16, x0 + 72, 31);
  u8g2.drawTriangle(x0 + 96, 0, x1 + 96, 16, x0 + 96, 31);
  u8g2.drawTriangle(x0 + 120, 0, x1 + 120, 16, x0 + 120, 31);
  u8g2.sendBuffer();
  //blockDisplayChangeUntil = millis() + 10UL;
#endif
}

void displayBigText(String text) {
#ifdef HAS_DISPLAY
  if (blockDisplayChangeUntil > millis()) {
    return;
  }

  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tf); // choose a suitable font
  u8g2.setFontDirection(0);
  u8g2.setCursor(0, 15);
  u8g2.print(text);
  u8g2.setCursor(64, 15);
  u8g2.print(text);
  u8g2.setCursor(0, 30);
  u8g2.print(text);
  u8g2.setCursor(64, 30);
  u8g2.print(text);
  u8g2.sendBuffer();

  blockDisplayChangeUntil = millis() + 200UL;
#endif
}

void showDistanceOnDisplay() {
  /*
    #ifdef HAS_DISPLAY
      if (blockDisplayChangeUntil > millis()) {
        return;
      }
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tf); // choose a suitable font
      u8g2.setFontDirection(0);
      u8g2.setCursor(0, 15);
      u8g2.print("Distance left  = ");
      u8g2.print(distanceLeft);
      u8g2.setCursor(0, 30);
      u8g2.print("Distance right = ");
      u8g2.print(distanceRight);
      u8g2.sendBuffer();

      blockDisplayChangeUntil = millis() + 300UL;
    #endif
  */
}

void showVoltageOnDisplay() {
  /*
    #ifdef HAS_DISPLAY
      if (blockDisplayChangeUntil > millis()) {
        return;
      }
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tf); // choose a suitable font
      u8g2.setFontDirection(0);
      u8g2.setCursor(0, 15);
      u8g2.print("Voltage is low!!!!!!");
      u8g2.setCursor(0, 30);
      u8g2.print("Current = ");
      u8g2.print(currentVoltage);
      u8g2.sendBuffer();
      blockDisplayChangeUntil = millis() + 20000UL; // refresh display every 20 seconds
    #endif
  */
}

void showCryingEyes() {
  /*
    #ifdef HAS_DISPLAY
      if (blockDisplayChangeUntil > millis()) {
        return;
      }
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tf); // choose a suitable font
      u8g2.setFontDirection(0);
      u8g2.setCursor(0, 15);
      u8g2.print("Voltage is low!!!!!!");
      u8g2.setCursor(0, 30);
      u8g2.print("Current = ");
      u8g2.print(currentVoltage);
      u8g2.sendBuffer();
      blockDisplayChangeUntil = millis() + 5000UL; // refresh display every 5 seconds
    #endif
  */
}

void printMp3Detail(uint8_t type, int value) {
#ifdef HAS_MP3
  switch (type) {
    case TimeOut:
      Console.println(F("MP3: Time Out!"));
      break;
    case WrongStack:
      Console.println(F("MP3: Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Console.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Console.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Console.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Console.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Console.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Console.print(F("Number:"));
      Console.print(value);
      Console.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Console.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Console.println(F("Card not found"));
          break;
        case Sleeping:
          Console.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Console.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Console.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Console.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Console.println(F("Cannot Find File"));
          break;
        case Advertise:
          Console.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
#endif
}
