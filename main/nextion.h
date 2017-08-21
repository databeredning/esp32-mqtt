#ifndef _NEXTION_H
#define _NEXTION_H
#include <stdint.h>

#define NEX_RET_CMD_FINISHED                (0x01)
#define NEX_RET_EVENT_LAUNCHED              (0x88)
#define NEX_RET_EVENT_UPGRADED              (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_INVALID_CMD                 (0x00)
#define NEX_RET_INVALID_COMPONENT_ID        (0x02)
#define NEX_RET_INVALID_PAGE_ID             (0x03)
#define NEX_RET_INVALID_PICTURE_ID          (0x04)
#define NEX_RET_INVALID_FONT_ID             (0x05)
#define NEX_RET_INVALID_BAUD                (0x11)
#define NEX_RET_INVALID_VARIABLE            (0x1A)
#define NEX_RET_INVALID_OPERATION           (0x1B)

typedef struct
{
  char SV[8];
  char Mode[8];
  char MaxOnTime[8];
  char MaxOffTime[8];
} nextion_config;


uint8_t nextion_init( void );
void    nextion_send_command( char *cmd );
uint8_t nextion_get_response( char * response );
uint8_t nextion_get_register( const char *reg, char *val );
uint8_t nextion_get_config_txt( const char *reg, char *val );
void    nextion_set_config_txt( const char *reg, const char *val );
void    nextion_set_status_txt( const char *reg, const char *val );

void    nextion_req_config_txt();
void    nextion_req_status_txt();
uint8_t nextion_handle_req_config_txt( nextion_config *config );

#endif
