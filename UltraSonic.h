#ifndef UltraSonic_h
  #define UltraSonic_h
  #include <Arduino.h>
  
  #define MAX_SENSOR_DISTANCE 500 // Maximum sensor distance can be as high as 500cm, no reason to wait for ping longer than sound takes to travel this distance and back. Default=500
  #define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57
  #define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance. Default=0
  #define MAX_SENSOR_DELAY 5800   // Maximum uS it takes for sensor to start the ping. Default=5800
  #define ECHO_TIMER_FREQ 24      // Frequency to check for a ping echo (every 24uS is about 0.4cm accuracy). Default=24
  #define PING_OVERHEAD 1
  #define PING_TIMER_OVERHEAD 1

  class UltraSonic {
    public:
      UltraSonic(uint8_t trigger_pin, uint8_t echo_pin, unsigned int max_cm_distance = MAX_SENSOR_DISTANCE);
      unsigned int ping(unsigned int max_cm_distance = 0);
      unsigned long ping_cm(unsigned int max_cm_distance = 0);
    private:
      boolean ping_trigger();
      void set_max_distance(unsigned int max_cm_distance);
      uint8_t _triggerPin;
      uint8_t _echoPin;
      unsigned int _maxEchoTime;
      unsigned long _max_time;
  };


#endif
