#ifndef _DBNODE_H
#define _DBNODE_H

#include <stdint.h>
#include "mqtt.h"
#include "pid.h"


// Status bits in node_runtime.status.curflag
#define NETWORK_STATUS 0b00000001

#define REG_SV 0           // Set value
#define REG_PV 1           // Process value
#define REG_MODE 2         // MANUAL / AUTO
#define REG_MAX_ON 3       // Max on timer
#define REG_MAX_OFF 4      // Max off timer
#define REG_HYST 5         // Hysteresis, 10 = 1/10 of degree

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
  mqtt_client *client_mqtt;
  int32_t blink_intervall;
  dio8_t status;
  dio8_t input;
  dio8_t output;
  aio8_16bit_t reg16;
} runtime_node;


typedef struct
{
  uint16_t   mode;            // Mode node COLD / RUN
  uint16_t   max_on;
  uint16_t   max_off;
  uint16_t   min_on;
  uint16_t   min_off;
  uint16_t   hysteresis;
  pidreg_t   pid;             // PID config
} config_node;


#endif
