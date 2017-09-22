#ifndef _PID_H
#define _PID_H

#include <stdint.h>

#define THERMOSTAT_MANUAL  0
#define THERMOSTAT_AUTO    1
#define THERMOSTAT_DIRECT  0
#define THERMOSTAT_REVERSE  1

typedef struct
{
  uint8_t mode;            // Controller mode ( auto/manual )
  uint8_t direction;       // Controller direction
  int16_t sv;              // Set value
  int16_t kp;              // PID constant
  int16_t ki;              // PID constant
  int16_t kd;              // PID constant
  int16_t out_max;         // Max output
  int16_t out_min;         // Min output
  int16_t sample_time;     // Loop time in seconds
  // runtime vars
  unsigned long last_time; //
  int16_t last_input;
  int16_t iterm;           // Derivative on Measurement
  int16_t output;          // output value
} pidreg_t;

void pid_compute( pidreg_t * tc, int16_t pv );

void pid_init( pidreg_t * tc, int16_t pv );

#endif
