#ifndef _DBNODE_H
#define _DBNODE_H

#include <stdint.h>

typedef struct
{
  uint8_t curflag;
  uint8_t newflag;
  uint8_t ackflag;
} dio8_t;

#endif
