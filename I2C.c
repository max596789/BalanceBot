/*
 * I2C.c
 *
 * Created: 1/12/2015 2:42:36 AM
 *  Author: Jacob
 */ 

#include "I2C.h"
#include <avr/io.h>
#include <stdint.h>
#include <util/twi.h>

void initI2C()
{
	TWCR |= (1<<TWEN);						//Enable the TWI/I2C interface
	TWBR = ((F_CPU / 400000L) - 16) / 2;	//Set the SCL frequency to 400,000; F_CPU is 20000000
}

void I2C_write(uint8_t slaveAddress, uint8_t reg, uint8_t data[], uint8_t dataLength, uint8_t registerSend)
{
	uint8_t status, i;
	
	status = I2C_start();				//Send start condition				
	if (status != TW_START)				//Check value of TWI Status Register. Mask prescaler bits. 
		ERROR(status);					//If status different from start go to error
	
	TWDR = (slaveAddress + TW_WRITE) << 1;//Load SLA_W into TWDR Register
	status = I2C_transmit();			//transmit
	if (status != TW_MT_SLA_ACK)		//Check value of TWI Status Register. Mask prescaler bits.
		ERROR(status);					//If status different from MT_SLA_ACK go to ERROR
	
	if (registerSend > 0)						//if writing to a register
	{
		TWDR = reg;						//Load register into TWDR Register.
		status = I2C_transmit();		//transmit
		if (status != TW_MT_DATA_ACK)	//Check value of TWI Status Register. Mask prescaler bits.
			ERROR(status);				//If status different from MT_DATA_ACK go to ERROR
	} 
	
	for (i = 0; i < dataLength; i++)
	{
		TWDR = data[i];					//Load DATA into TWDR Register.
		status = I2C_transmit();		//transmit
		if (status != TW_MT_DATA_ACK)	//Check value of TWI Status Register. Mask prescaler bits.
			ERROR(status);				//If status different from MT_DATA_ACK go to ERROR
	}
	
	I2C_stop();							//Send stop condition
}

void I2C_recieve(uint8_t slaveAddress, uint8_t regAddress, uint8_t data[], uint8_t dataLength)
{
	uint8_t status, i;
	
	status = I2C_start();				//Send start condition
	if (status != TW_START)				//Check value of TWI Status Register. Mask prescaler bits.
		ERROR(status);					//If status different from start go to error
		
	TWDR = (slaveAddress + TW_WRITE) << 1;		//Load SLA_W into TWDR Register
	status = I2C_transmit();			//transmit
	if (status != TW_MT_SLA_ACK)		//Check value of TWI Status Register. Mask prescaler bits.
		ERROR(status);					//If status different from MR_SLA_ACK go to ERROR
	
	TWDR = regAddress;					//Load register address into TWDR Register
	status = I2C_transmit();			//transmit
	if (status != TW_MT_SLA_ACK)		//Check value of TWI Status Register. Mask prescaler bits.
		ERROR(status);					//If status different from MR_SLA_ACK go to ERROR
	
	status = I2C_start();				//Send start condition
	if (status != TW_START)				//Check value of TWI Status Register. Mask prescaler bits.
		ERROR(status);					//If status different from start go to error
	
	TWDR = (slaveAddress << 1) + TW_READ;		//Load SLA_R into TWDR Register
	status = I2C_transmit();			//transmit
	if (status != TW_MR_SLA_ACK)		//Check value of TWI Status Register. Mask prescaler bits.
		ERROR(status);					//If status different from MR_SLA_ACK go to ERROR
	
	status = I2C_transmit();		//transmit
	if (status != TW_MR_DATA_ACK)	//Check value of TWI Status Register. Mask prescaler bits.
		ERROR(status);				//If status different from MR_SLA_ACK go to ERROR
	data[0] = TWDR;

	//TWCR &= ~(1<<TWEA);					//generate a NACK to let the slave now that the master is done
	I2C_stop();							//Send stop condition
}

uint8_t I2C_start(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //start protocol
	while (!(TWCR & (1 << TWINT)));			//Wait for TWINT Flag set. This	indicates that the START condition has been transmitted
	return (TWSR & 0xF8);					//Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
}

uint8_t I2C_transmit(void)
{
	TWCR = (1<<TWINT) |	(1<<TWEN);			//Clear TWINT bit in TWCR to start transmission of address
	while (!(TWCR & (1 << TWINT)));			//Wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received
	return (TWSR & 0xF8);					//Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
}

uint8_t I2C_transmit_ACK(void)
{
	TWCR = (1<<TWINT) |	(1<<TWEN) | (1<<TWEA);	//Clear TWINT bit in TWCR to start transmission of address
	while (!(TWCR & (1 << TWINT)));			//Wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received
	return (TWSR & 0xF8);					//Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
}

void I2C_stop(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	//stop transmission
}

void ERROR(uint8_t error)
{
	
}