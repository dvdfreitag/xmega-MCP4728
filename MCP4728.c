#include <avr/io.h>

#include "MCP4728.h"
#include "SoftwareTWI.h"

#ifdef _HARDWARE_TWI_
	#include "TWI.h"
#endif

#define DEVICE_CODE		0xC0U

#define GENERAL_CALL 	0x00U
#define RESET			0x06U
#define WAKEUP			0x09U
#define UPDATE			0x08U
#define READADDRESS		0x0CU
#define READRESTART		0xC1U

#define FASTWRITE		0x00U
#define MULTIWRITE		0x40U
#define SEQWRITE		0x50U
#define SINGLEWRITE		0x58U

#define WRITECURRENT	0x61U
#define WRITENEW		0x62U
#define WRITECONFIRM	0x63U

#define WRITEVREF		0x80U
#define WRITEPD			0xA0U
#define WRITEGAIN		0xC0U

#define NOP asm volatile("nop");

#ifndef _HARDWARE_TWI_
	#define twi_start	STWI_Start
	#define twi_restart	STWI_Restart
	#define twi_stop	STWI_Stop

	#define twi_write	STWI_WriteByte
	#define twi_read	STWI_ReadByte

	#define twi_ack		STWI_ACK
	#define twi_nack	STWI_NACK

	PORT_t *TWI;
	uint8_t SDA;
	uint8_t SCL;
#else
	#define twi_start	TWI_Start
	#define twi_restart	TWI_Restart
	#define twi_stop	TWI_Stop

	#define twi_write	TWI_WriteByte
	#define twi_read	TWI_ReadByte

	#define twi_ack		TWI_ACK
	#define twi_nack	TWI_NACK

	TWI_t *TWI;
#endif

PORT_t *LDACPort;
uint8_t LDAC;

uint8_t Address;
uint8_t AutoUpdate;

uint8_t Buffer[4][2][3]; // 4 channels, input/eeprom, 3 data bytes

uint8_t general_call(uint8_t code)
{
	if (!TWI) return twi_nack;
	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif
	
	if (twi_write(GENERAL_CALL) != twi_ack) goto error;
	if (twi_write(code) != twi_ack) goto error;

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t twi_write_ldac(PORT_t *twi, uint8_t sda, uint8_t scl, uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (data & 0x80)
		{
			twi->DIRCLR = sda;
		}
		else
		{
			twi->DIRSET = sda;
		}
		
		data <<= 1;
		
		NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
		NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
		twi->DIRCLR = scl;
		NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
		NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
		twi->DIRSET = scl;
	}
	
	twi->DIRCLR = sda;
	LDACPort->OUTCLR = LDAC;
	NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
	NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
	twi->DIRCLR = scl;
	data = twi->IN & sda;
	NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
	NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP
	twi->DIRSET = scl;
	
	return data;
}

uint8_t MCP4728_Reset(void)
{
	return general_call(RESET);
}

uint8_t MCP4728_WakeUp(void)
{
	return general_call(WAKEUP);
}

uint8_t MCP4728_Update(void)
{
	return general_call(UPDATE);
}

#ifndef _HARDWARE_TWI_
	uint8_t MCP4728_ReadAddress(void)
#else
	uint8_t MCP4728_ReadAddress(PORT_t *twi, uint8_t sda, uint8_t scl)
#endif
{
	if (!TWI || !LDACPort) return twi_nack;
	
	LDACPort->OUTSET = LDAC;
	
	// Generate START condition
	STWI_Start(twi, sda, scl);
	// Write General Call
	if (STWI_WriteByte(GENERAL_CALL) != twi_ack) 
	{
		goto error;
	}
	// Write old address with LDAC transition
	if (twi_write_ldac(twi, sda, scl, READADDRESS) != twi_ack)
	{
		goto error;
	}
	// Generate RESTART condition
	STWI_Restart();
	// Write restart byte
	if (STWI_WriteByte(READRESTART) != twi_ack)
	{
		goto error;
	}
	// Read response
	uint8_t data = STWI_ReadByte(twi_nack);

	STWI_Stop();
	LDACPort->OUTSET = LDAC;
	return data;

error:
	STWI_Stop();
	LDACPort->OUTSET = LDAC;
	return 0xFF;
}

uint8_t MCP4728_FastWrite(uint8_t *data, uint8_t length)
{
	if (!TWI) return twi_nack;
	if (length < 1)  return twi_nack;
	uint8_t *ptr = data;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif

	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;

	for (uint8_t i = 0; i < length; i++)
	{
		if (twi_write(FASTWRITE | (*ptr++ & 0x3F)) != twi_ack) goto error;

		for (uint8_t j = 0; j < 7; j++)
		{
			if (twi_write(*ptr++) != twi_ack) goto error;
		}
	}

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_WriteAll(uint16_t value)
{
	if (!TWI) return twi_nack;
	uint8_t *ptr = (uint8_t *)&value;
	
	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif
	
	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	
	for (uint8_t i = 0; i < 4; i++)
	{
		uint8_t vref = MCP4728_GetVREF(i, 0);
		uint8_t gain = MCP4728_GetGain(i, 0);
		
		if (twi_write(MULTIWRITE | (i << 1))) goto error;
		if (twi_write((ptr[1] & 0x0F) | vref | gain) != twi_ack) goto error;
		if (twi_write(ptr[0]) != twi_ack) goto error;
	}
	
	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_MultiWrite(uint8_t *data, uint8_t length)
{
	if (!TWI) return twi_nack;
	if ((length < 3) || ((length % 3) != 0)) return twi_nack;
	uint8_t *ptr = data;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif

	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	if (twi_write(MULTIWRITE | (*ptr++ & 0x07)) != twi_ack) goto error;

	for (uint8_t i = 1; i < length; i++)
	{
		if (twi_write(*ptr++) != twi_ack) goto error;
	}

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_SingleWriteFast(uint8_t *data, uint8_t channel)
{
	if (!TWI) return twi_nack;
	if (channel > 3) return twi_nack;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif
	
	uint8_t vref = MCP4728_GetVREF(channel, 0);
	uint8_t gain = MCP4728_GetGain(channel, 0);
	
	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	if (twi_write(MULTIWRITE | (channel << 1)) != twi_ack) goto error;
	if (twi_write((data[1] & 0x0F) | vref | gain) != twi_ack) goto error;
	if (twi_write(data[0]) != twi_ack) goto error;
	
	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_SequentialWrite(uint8_t start, uint8_t *data)
{
	if (!TWI) return twi_nack;
	if (start > 3) return twi_nack;
	uint8_t *ptr = data;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif

	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	if (twi_write(SEQWRITE | (start++ << 1)) != twi_ack) goto error;

	for (uint8_t i = 0; i < (8 - (start * 2)); i++)
	{
		if (twi_write(*ptr++) != twi_ack) goto error;
	}

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_SingleWrite(uint8_t channel, uint16_t data)
{
	if (!TWI) return twi_nack;
	if (channel > 3) return twi_nack;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif

	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	if (twi_write(SINGLEWRITE | (channel << 1)) != twi_ack) goto error;
	if (twi_write((uint8_t)(data >> 8)) != twi_ack) goto error;
	if (twi_write((uint8_t)(data & 0x00FF)) != twi_ack) goto error;

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

#ifndef _HARDWARE_TWI_
	uint8_t MCP4728_WriteAddress(uint8_t new)
#else
	uint8_t MCP4728_WriteAddress(PORT_t *twi, uint8_t sda, uint8_t scl, uint8_t new)
#endif
{
	if (!TWI || !LDACPort) return twi_nack;

	LDACPort->OUTSET = LDAC;
	
	// Generate START condition
	STWI_Start(twi, sda, scl);
	// Write device code with current address bits
	if (STWI_WriteByte(DEVICE_CODE | Address) != twi_ack) goto error;
	// Write command type with current address bits and LDAC strobe
	if (twi_write_ldac(twi, sda, scl, WRITECURRENT | (Address << 1)) != twi_ack) goto error;
	
	MCP4728_SetAddress(new);

	// Write command type with new address bits
	if (STWI_WriteByte(WRITENEW | (Address << 1)) != twi_ack) goto error;
	// Write command type with new address confirmation
	if (STWI_WriteByte(WRITECONFIRM | (Address << 1)) != twi_ack) goto error;

	STWI_Stop();
	LDACPort->OUTSET = LDAC;
	return twi_ack;

error:
	STWI_Stop();
	LDACPort->OUTSET = LDAC;
	return twi_nack;
}

uint8_t MCP4728_WriteVREF(uint8_t vref)
{
	if (!TWI) return twi_nack;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif

	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	if (twi_write(WRITEVREF | (vref & 0x0F)) != twi_ack) goto error;

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_WritePD(uint8_t pd)
{
	if (!TWI) return twi_nack;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif	
	
	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	if (twi_write(WRITEPD | ((pd >> 4) & 0x0F)) != twi_ack) goto error;
	if (twi_write(pd << 4) != twi_ack) goto error;

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_WriteGain(uint8_t gain)
{
	if (!TWI) return twi_nack;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif	
	
	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	if (twi_write(WRITEGAIN | (gain & 0x0F)) != twi_ack) goto error;

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

uint8_t MCP4728_ReadConfig()
{
	if (!TWI) return twi_nack;
	uint8_t *ptr = (uint8_t *)Buffer;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif	
	
	if (twi_write(DEVICE_CODE | Address | 0x01) != twi_ack) goto error;
	
	for (uint8_t i = 1; i < (4 * 3 * 2); i++)
	{
		*ptr++ = twi_read(twi_ack);
	}

	*ptr++ = twi_read(twi_nack);

	twi_stop(twi_ack);
	return twi_ack;

error:
	twi_stop(twi_nack);
	return twi_nack;
}

#ifndef _HARDWARE_TWI_
	void MCP4728_Init(PORT_t *twi, uint8_t sda, uint8_t scl, PORT_t *ldacPort, uint8_t ldac, uint8_t address)
#else
	void MCP4728_Init(TWI_t *twi, PORT_t *ldacPort, uint8_t ldac, uint8_t address)
#endif
{
	TWI = twi;
	
	#ifndef _HARDWARE_TWI_
		SDA = sda;
		SCL = scl;
	#endif

	LDACPort = ldacPort;
	LDAC = ldac;
	Address = (address << 1) & 0x07;
}

#ifndef _HARDWARE_TWI_
	void MCP4728_SetPort(PORT_t *twi, uint8_t sda, uint8_t scl)
#else
	void MCP4728_SetPort(TWI_t *twi)
#endif
{
	TWI = twi;
	
	#ifndef _HARDWARE_TWI_
		SDA = sda;
		SCL = scl;
	#endif
}

void MCP4728_SetLDAC(PORT_t *port, uint8_t ldac)
{
	LDACPort = port;
	LDAC = ldac;
}

uint8_t MCP4728_GetReady(uint8_t channel, uint8_t eeprom)
{
	if (channel > 3 || eeprom > 1) return 0x00;
	return Buffer[channel][eeprom][0] & 0x80;
}

uint8_t MCP4728_GetPOR(uint8_t channel, uint8_t eeprom)
{
	if (channel > 3 || eeprom > 1) return 0x00;
	return Buffer[channel][eeprom][0] & 0x40;
}

void MCP4728_SetAddress(uint8_t address)
{
	Address = (address << 1) & 0x07;
}

uint8_t MCP4728_GetAddress(void)
{
	return Address;
}

uint8_t MCP4728_GetVREF(uint8_t channel, uint8_t eeprom)
{
	if (channel > 3 || eeprom > 1) return 0x00;
	return Buffer[channel][eeprom][1] & 0x80;
}

uint8_t MCP4728_GetPD(uint8_t channel, uint8_t eeprom)
{
	if (channel > 3 || eeprom > 1) return 0x00;
	return Buffer[channel][eeprom][1] & 0x60;
}

uint8_t MCP4728_GetGain(uint8_t channel, uint8_t eeprom)
{
	if (channel > 3 || eeprom > 1) return 0x00;
	return Buffer[channel][eeprom][1] & 0x10;
}

uint16_t MCP4728_GetValue(uint8_t channel, uint8_t eeprom)
{
	if (channel > 3 || eeprom > 1) return 0x00;
	return ((uint16_t)(Buffer[channel][eeprom][1] & 0x0F) << 8) | (Buffer[channel][eeprom][2]);
}
