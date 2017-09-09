/**
 * PID process
 *
**/

//#define DEBUG

#include <stdio.h>
#include <time.h>
#include "pid.h"

const char *PID_TAG = "PID";

void pid_compute( pidreg_t * tc, int16_t sv, int16_t pv )
{
  /*
   int16_t output;

   unsigned long now = clock_seconds();
   uint16_t delta_time = now - tc->last_time;

#ifdef DEBUG
   printf("DEBUG:  [PID] mode  : %s\n", ( tc->mode == THERMOSTAT_MANUAL ? "MANUAL" : "AUTO" ));
#endif
   if( tc->mode == THERMOSTAT_MANUAL )
     return;

   if( delta_time >= tc->sample_time )
   {
      int16_t error = sv - pv;

      tc->iterm += ( tc->ki * error);

      if(tc->iterm > tc->out_max)
        tc->iterm = tc->out_max;
      else if( tc->iterm < tc->out_min)
        tc->iterm = tc->out_min;

      output = tc->kp * error + tc->iterm - tc->kd * ( pv - tc->last_input);

      if( output > tc->out_max)
        output = tc->out_max;
      else if( output < tc->out_min)
        output = tc->out_min;

      tc->last_input = pv;
      tc->last_time = now;
      tc->output = output;
#ifdef DEBUG
      printf("              SV    : %d\n", sv );
      printf("              PV    : %d\n", pv );
      printf("              K     : P(%d)\n", tc->kp );
      printf("              error : %d\n", sv - pv );
      printf("              output: %d\n", tc->output );
#endif

   }
   */
}

void pid_init( pidreg_t * tc, int16_t pv )
{
   tc->last_input = pv;
   tc->iterm = 0;
   tc->last_time = 0;
}
