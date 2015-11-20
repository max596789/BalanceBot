/*
 * clock_2.h
 *
 * Created: 3/16/2015 7:09:28 PM
 *  Author: ECE User
 */ 


#ifndef CLOCK_2_H_
#define CLOCK_2_H_

#include <stdint.h>

typedef enum  
{
	no_pre_2,
	pre_8_2,
	pre_32_2,
	pre_64_2,
	pre_128_2,
	pre_256_2,
	pre_1024_2
}prescaler_2;

uint16_t prescaleValue; 
prescaler_2 prescaleSetting_2;
volatile uint16_t numberOfpulsesOC2A;
volatile uint16_t numberOfpulsesOC2B;
volatile uint16_t pulseWantedOC2A;
volatile uint16_t pulseWantedOC2B;

void initOC2A(int pin);
void initOC2B(int pin);
void setClockPrescaler_2(prescaler_2 prescale);
void sendClockPulses_2(uint16_t pulsesOC2A, uint16_t pulsesOC2B);
void sendOC2AClockPulses(uint16_t numberOfPulses);
void sendOC2BClockPulses(uint16_t numberOfPulses);
void changeOC2AFrequency(uint32_t frequency);
void changeOC2BDutyCycle(uint32_t frequency);
void disconnectOC2A();
void disconnectOC2B();
void toggleOC2A();
void toggleOC2B();
void stopClocks_2();

#endif /* CLOCK_2_H_ */