#include <avr/io.h>

#include "MCP4728.h"

#ifndef _HARDWARE_TWI_
	#include "SoftwareTWI.h"
#else
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

#define WRITEADDRESS	0x60U
#define WRITECURRENT	0x61U
#define WRITENEW		0x62U
#define WRITECONFIRM	0x63U

#define WRITEVREF		0x80U
#define WRITEPD			0xA0U
#define WRITEGAIN		0xC0U

#define NOP asm volatile("nop");
// Quarter-bit delay
void qdelay(void) { NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP }
// Half-bit delay
void hdelay(void) { qdelay(); qdelay(); }

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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
	return twi_nack;
}

uint8_t twi_write_ldac(PORT_t *twi, uint8_t sda, uint8_t scl, uint8_t data)
{
	for (int8_t i = 7; i >= 0; i--)
	{// Write the data bit
		if (data & (1 << i))
		{
			twi->DIRCLR = sda; // Set SDA high
		}
		else
		{
			twi->DIRSET = sda;	// Set SDA low
		}
		// One clock transition
		hdelay();
		twi->DIRCLR = scl;	// Set SCL high
		hdelay();
		twi->DIRSET = scl;	// Set SCL low
	}
	// Ensure SDA is "high" -> SDA is an input.
	// This is where the slave will indicate an ACK by actively pulling SDA down,
	//   or a NACK is indicated by no response.
	twi->DIRCLR = sda;	// Set SDA high
	LDACPort->OUTCLR = LDAC; // Set LDAC low
	hdelay();
	twi->DIRCLR = scl;	// Set SCL high
	data = twi->IN & sda; // Get ACK bit
	hdelay();
	twi->DIRSET = scl;	// Set SCL low
	hdelay();

	return (data != 0);
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
	uint8_t MCP4728_ReadAddress(PORT_t *twi, uint8_t sda, uint8_t sda)
#endif
{
	if (!TWI || !LDACPort) return twi_nack;
	
	LDACPort->OUTSET = LDAC;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif

	if (twi_write(GENERAL_CALL) != twi_ack) goto error;
	
	#ifndef _HARDWARE_TWI_
		if (twi_write_ldac(TWI, SDA, SCL, READADDRESS) != twi_ack) goto error;
	#else
		TWI->MASTER.CTRLA &= ~TWI_MASTER_ENABLE_bm;
		if (twi_write_ldac(twi, sda, scl, READADDRESS) != twi_ack) goto error;
		TWI->MASTER.CTRLA |= TWI_MASTER_ENABLE_bm;
	#endif

	twi_restart();

	if (twi_write(READRESTART) != twi_ack) goto error;
	uint8_t address = twi_read(twi_nack);

	twi_stop();
	LDACPort->OUTSET = LDAC;
	return address;

error:
	twi_stop();
	LDACPort->OUTSET = LDAC;
	return twi_nack;
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
	return twi_nack;
}

#ifndef _HARDWARE_TWI_
	uint8_t MCP4728_WriteAddress(uint8_t new)
#else
	uint8_t MCP4728_WriteAddress(PORT_t *twi, uint8_t sda, uint8_t sda, uint8_t new)
#endif
{
	if (!TWI || !LDACPort) return twi_nack;

	LDACPort->OUTSET = LDAC;

	#ifndef _HARDWARE_TWI_
		twi_start(TWI, SDA, SCL);
	#else
		twi_start(TWI);
	#endif

	if (twi_write(DEVICE_CODE | Address) != twi_ack) goto error;
	
	#ifndef _HARDWARE_TWI_
		if (twi_write_ldac(TWI, SDA, SCL, WRITEADDRESS | (Address << 1)) != twi_ack) goto error;
	#else
		TWI->MASTER.CTRLA &= ~TWI_MASTER_ENABLE_bm;
		if (twi_write_ldac(twi, sda, scl, WRITEADDRESS | (Address << 1)) != twi_ack) goto error;
		TWI->MASTER.CTRLA |= TWI_MASTER_ENABLE_bm;
	#endif

	Address = (new << 1) & 0x07;

	if (twi_write(WRITENEW | (Address << 1)) != twi_ack) goto error;
	if (twi_write(WRITECONFIRM | (Address << 1)) != twi_ack) goto error;

	twi_stop();
	LDACPort->OUTSET = LDAC;
	return twi_ack;

error:
	twi_stop();
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
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

	twi_stop();
	return twi_ack;

error:
	twi_stop();
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
	Address = (address & 0x07) << 1;
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
	Address = (address & 0x07) << 1;
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
