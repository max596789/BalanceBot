/*
 * clock.h
 *
 * Created: 1/12/2015 2:57:33 AM
 *  Author: Jacob
 */ 


#ifndef CLOCK_0_H_
#define CLOCK_0_H_

#include <stdint.h>

typedef enum  
{
	no_pre_0,
	pre_8_0,
	pre_64_0,
	pre_256_0,
	pre_1024_0
}prescaler;

uint16_t prescaleValue; 
prescaler prescaleSetting_0;
volatile uint16_t numberOfPulsesOC0A;
volatile uint16_t numberOfPulsesOC0B;
volatile uint16_t pulseWantedOC0A;
volatile uint16_t pulseWantedOC0B;

void initOC0A(int pin);
void initOC0B(int pin);
void setClockPrescaler_0(prescaler prescale);
void sendClockPulses_0(uint16_t pulsesOC0A, uint16_t pulsesOC0B);
void sendOC0AClockPulses(uint16_t numberOfPulses);
void sendOC0BClockPulses(uint16_t numberOfPulses);
void changeOC0AFrequency(uint32_t frequency);
void changeOC0BDutyCycle(uint32_t frequency);
void disconnectOC0A();
void disconnectOC0B();
void toggleOC0A();
void toggleOC0B();
void stopClocks();

#endif /* CLOCK_H_ */