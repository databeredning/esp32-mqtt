/**
 * PID process
 *
**/

//#define DEBUG

#include <stdio.h>
#include <time.h>
#include "pid.h"
#include "esp_log.h"

const char *PID_TAG = "PID";


// Must be called on a fixed intervall as the code
// don't measure the intervall between every iteration.
void pid_compute( pidreg_t * tc, int16_t pv )
{
   int16_t output;
   int16_t error;

  //unsigned long now = clock_seconds();
  //uint16_t delta_time = now - tc->last_time;
  //if( delta_time < tc->sample_time )
  //  return;

  if(tc->mode == THERMOSTAT_MANUAL)
    return;

  error = tc->sv - pv;

  tc->iterm += (tc->ki * error);

  if(tc->iterm > tc->out_max)
    tc->iterm = tc->out_max;
  else if( tc->iterm < tc->out_min)
    tc->iterm = tc->out_min;

  output = tc->kp * error + tc->iterm - tc->kd * (pv - tc->last_input);

  if(output > tc->out_max)
    output = tc->out_max;
  else if(output < tc->out_min)
    output = tc->out_min;

  tc->last_input = pv;
  //tc->last_time = now;
  tc->output = output;

  //ESP_LOGI(PID_TAG,"pid_compute() output = %d", output);

}

void pid_init( pidreg_t * tc, int16_t pv )
{
   tc->last_input = pv;
   tc->iterm = 0;
   //tc->last_time = 0;
}
