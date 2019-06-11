#include "Arduino.h"
#include "HCSR04.h"

HCSR04::HCSR04(uint8_t triggerPin, uint8_t echoPin, uint8_t maxDistance) {
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
	this->maxDistance = maxDistance;
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

unsigned int HCSR04::ping() {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH);
    unsigned int distance = duration / 2.0 * 0.0343;
	
    if (distance > 3000) {
         return 0;
    } else if (distance > maxDistance && distance < 300) {
		return maxDistance;
	} else if (distance > 0 && distance <= maxDistance) {
        return distance;
    }
}

unsigned int HCSR04::ping_median(uint8_t it) {
	unsigned int uS[it], last;
	uint8_t j, i = 0;
	unsigned long t;
	uS[0] = 0;

	while (i < it) {
		t = micros();                  // Start ping timestamp.
		last = ping();  				// Send ping.

		if (i > 0) {               // Don't start sort till second ping.
			for (j = i; j > 0 && uS[j - 1] < last; j--) // Insertion sort loop.
				uS[j] = uS[j - 1];                      // Shift ping array to correct position for sort insertion.
		} else j = 0;              // First ping is sort starting point.
		uS[j] = last;              // Add last ping to array in sorted position.
		i++;                       // Move to next ping.

		if (i < it && micros() - t < 29000)
			delay((29000 + t - micros()) / 1000); // Millisecond delay between pings.

	}
	return (uS[it >> 1]); // Return the ping distance median.
}
