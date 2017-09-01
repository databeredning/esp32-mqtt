/**
 * Simple modbus process
 *
**/
#ifndef _MODBUS_H_
#define _MODBUS_H_

#include <stdint.h>

#define BUFFER_SIZE 128

char          frame[BUFFER_SIZE];
uint16_t      modbus_register[3];

void    modbus_task(void *pvParameter);

unsigned int  calculateCRC(unsigned char bufferSize);
unsigned int  readholdingregister( unsigned char id, unsigned int address, unsigned char numreg );
unsigned char getresponse();


#endif
