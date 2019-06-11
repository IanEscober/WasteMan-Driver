#ifndef HCSR04_H
#define HCSR04_H

#include "Arduino.h"

class HCSR04{
 public:
    HCSR04(uint8_t triggerPin, uint8_t echoPin, uint8_t maxDistance);
    unsigned int ping();
	unsigned int ping_median(uint8_t it);
	
 private:
    uint8_t triggerPin, echoPin;
	uint8_t maxDistance;
};

#endif
