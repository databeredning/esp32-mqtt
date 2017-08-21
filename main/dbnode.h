#ifndef _DBNODE_H
#define _DBNODE_H

#include <stdint.h>
#include "mqtt.h"

typedef struct
{
  uint8_t curflag;
  uint8_t newflag;
  uint8_t ackflag;
} dio8_t;

typedef struct
{
  uint8_t curflag;
  uint8_t newflag;
  uint8_t ackflag;
  int16_t curval[8];
} aio8_16bit_t;

typedef struct
{
  ip4_addr_t ip_addr;
  mqtt_client *client;
  int32_t blink_intervall;
  dio8_t input;
  dio8_t output;
  aio8_16bit_t analog;  
} node_runtime;

#endif
