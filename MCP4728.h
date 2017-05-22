#ifndef MCP4728_H_
#define MCP4728_H_

#include <avr/io.h>

// General call commands
uint8_t MCP4728_Reset(void);
uint8_t MCP4728_WakeUp(void);
uint8_t MCP4728_Update(void);
#ifndef _HARDWARE_TWI_
	uint8_t MCP4728_ReadAddress(void);
#else
	uint8_t MCP4728_ReadAddress(PORT_t *twi, uint8_t sda, uint8_t scl);
#endif
// Write commands
uint8_t MCP4728_FastWrite(uint8_t *data, uint8_t length);
uint8_t MCP4728_WriteAll(uint16_t value);
uint8_t MCP4728_MultiWrite(uint8_t *data, uint8_t length);
uint8_t MCP4728_SingleWriteFast(uint8_t *data, uint8_t channel);
uint8_t MCP4728_SequentialWrite(uint8_t start, uint8_t *data);
uint8_t MCP4728_SingleWrite(uint8_t channel, uint16_t data);
#ifndef _HARDWARE_TWI_
	uint8_t MCP4728_WriteAddress(uint8_t new);
#else
	uint8_t MCP4728_WriteAddress(PORT_t *twi, uint8_t sda, uint8_t scl, uint8_t new);
#endif
uint8_t MCP4728_WriteVREF(uint8_t vref);
uint8_t MCP4728_WritePD(uint8_t pd);
uint8_t MCP4728_WriteGain(uint8_t gain);
// Read commands
uint8_t MCP4728_ReadConfig();
// Utility functions
#ifndef _HARDWARE_TWI_
	void MCP4728_Init(PORT_t *twi, uint8_t sda, uint8_t scl, PORT_t *ldacPort, uint8_t ldac, uint8_t address);
	void MCP4728_SetPort(PORT_t *twi, uint8_t sda, uint8_t scl);
#else
	void MCP4728_Init(TWI_t *twi, PORT_t *ldacPort, uint8_t ldac, uint8_t address);
	void MCP4728_SetPort(TWI_t *twi);
#endif
void MCP4728_SetLDAC(PORT_t *port, uint8_t ldac);
// Config getter functions
uint8_t MCP4728_GetReady(uint8_t channel, uint8_t eeprom);
uint8_t MCP4728_GetPOR(uint8_t channel, uint8_t eeprom);
uint8_t MCP4728_GetAddress(void);
void MCP4728_SetAddress(uint8_t address);
uint8_t MCP4728_GetVREF(uint8_t channel, uint8_t eeprom);
uint8_t MCP4728_GetPD(uint8_t channel, uint8_t eeprom);
uint8_t MCP4728_GetGain(uint8_t channel, uint8_t eeprom);
uint16_t MCP4728_GetValue(uint8_t channel, uint8_t eeprom);

#endif
