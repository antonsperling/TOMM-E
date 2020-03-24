#include "UltraSonic.h"

UltraSonic::UltraSonic(uint8_t trigger_pin, uint8_t echo_pin, unsigned int max_cm_distance) {
  _triggerPin = trigger_pin;
  _echoPin = echo_pin;

  set_max_distance(max_cm_distance); // Call function to set the max sensor distance.

  pinMode(echo_pin, INPUT);     // Set echo pin to input (on Teensy 3.x (ARM), pins default to disabled, at least one pinMode() is needed for GPIO mode).
  pinMode(trigger_pin, OUTPUT); // Set trigger pin to output (on Teensy 3.x (ARM), pins default to disabled, at least one pinMode() is needed for GPIO mode).
}

unsigned int UltraSonic::ping(unsigned int max_cm_distance) {
  if (max_cm_distance > 0) set_max_distance(max_cm_distance); // Call function to set a new max sensor distance.

  if (!ping_trigger()) return NO_ECHO; // Trigger a ping, if it returns false, return NO_ECHO to the calling function.

  while (digitalRead(_echoPin))                 // Wait for the ping echo.
    if (micros() > _max_time) return NO_ECHO; // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.

  return (micros() - (_max_time - _maxEchoTime) - PING_OVERHEAD); // Calculate ping time, include overhead.
}

unsigned long UltraSonic::ping_cm(unsigned int max_cm_distance) {
  unsigned long echoTime = UltraSonic::ping(max_cm_distance); // Calls the ping method and returns with the ping echo distance in uS.
  return (echoTime / US_ROUNDTRIP_CM);              // Call the ping method and returns the distance in centimeters (no rounding).
}

void UltraSonic::set_max_distance(unsigned int max_cm_distance) {
  _maxEchoTime = min(max_cm_distance + 1, (unsigned int) MAX_SENSOR_DISTANCE + 1) * US_ROUNDTRIP_CM; // Calculate the maximum distance in uS (no rounding).
}

boolean UltraSonic::ping_trigger() {
  
  digitalWrite(_triggerPin, LOW);   // Set the trigger pin low, should already be low, but this will make sure it is.
  delayMicroseconds(4);             // Wait for pin to go low.
  digitalWrite(_triggerPin, HIGH);  // Set trigger pin high, this tells the sensor to send out a ping.
  delayMicroseconds(10);            // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
  digitalWrite(_triggerPin, LOW);   // Set trigger pin back to low.


  if (digitalRead(_echoPin)) return false;                // Previous ping hasn't finished, abort.
  _max_time = micros() + _maxEchoTime + MAX_SENSOR_DELAY; // Maximum time we'll wait for ping to start (most sensors are <450uS, the SRF06 can take up to 34,300uS!)
  while (!digitalRead(_echoPin))                          // Wait for ping to start.
    if (micros() > _max_time) return false;             // Took too long to start, abort.

  _max_time = micros() + _maxEchoTime; // Ping started, set the time-out.
  return true;                         // Ping started successfully.
}
