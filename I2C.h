/*
 * I2C.h
 *
 * Created: 1/12/2015 2:42:49 AM
 *  Author: Jacob
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>
#include <stdint.h>

#define F_CPU 20000000UL

void initI2C(void);
uint8_t I2C_start(void);
uint8_t I2C_transmit(void);
void I2C_stop(void);
void I2C_write(uint8_t slaveAddress, uint8_t reg, uint8_t data[], uint8_t dataLength,  uint8_t registerSend);
void I2C_recieve(uint8_t slaveAddress, uint8_t regAddress, uint8_t data[], uint8_t dataLength);
void ERROR(uint8_t error);  //need to finish error

#endif /* I2C_H_ */