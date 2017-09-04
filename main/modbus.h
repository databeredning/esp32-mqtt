/**
 * Simple modbus process
 *
**/
#ifndef _MODBUS_H_
#define _MODBUS_H_

#include <stdint.h>

// Public variables
uint16_t    modbus_register[3];

// Public functions
void    modbus_task(void *pvParameter);
uint8_t read_holding_registers( uint8_t id, uint16_t address, uint16_t numreg );
uint8_t preset_single_register( uint8_t id, uint16_t address, uint16_t value );


#endif
