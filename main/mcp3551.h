/**
 * SPI task to read advalue from MCP3551
 *
**/
#ifndef _MCP3551_H_
#define _MCP3551_H_

#include <stdint.h>

// Public variables
int32_t    mcp3551_value;

// Public functions
void    mcp3551_task(void *pvParameter);

#endif
