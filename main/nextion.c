#include <stdio.h>
#include <string.h>
#include "nextion.h"
#include "driver/uart.h"

void nextion_set_status_txt( const char *reg, const char *val )
{
  char tx[32];

  sprintf( tx, "status.%s.txt=\"%s\"", reg, val );
  nextion_send_command( tx );
}

void nextion_set_config_txt( const char *reg, const char *val )
{
  char tx[32];

  sprintf( tx, "config.%s.txt=\"%s\"", reg, val );
  nextion_send_command( tx );
}

void nextion_send_buff( char *buff, int len )
{
  uart_write_bytes( UART_NUM_1, (const char*)buff, len);
}

void nextion_send_command( char* cmd )
{
  char buffer[32];

  sprintf(buffer,"%s%c%c%c", cmd, 0xFF, 0xFF, 0xFF );
  uart_write_bytes( UART_NUM_1, buffer, strlen(buffer) );
}

uint8_t nextion_init( void )
{

  nextion_send_command("");
  nextion_send_command("bkcmd=1");
  nextion_send_command("page 0");

  return 0;
}

void nextion_req_config_txt()
{
  char tx[255];

  sprintf( tx,"printh 02\xFF\xFF\xFF"
              "print config.SV.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "print config.MaxOnTime.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "print config.MaxOffTime.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "print config.Mode.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "printh 03\xFF\xFF\xFF" );

  if( strlen( tx ) < 255 )
    nextion_send_buff( tx, strlen( tx ) );
}

void nextion_req_status_txt()
{
  char tx[255];

  sprintf( tx,"printh 02\xFF\xFF\xFF"
              "print status.SV.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "print status.MaxOnTime.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "print status.MaxOffTime.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "print status.Mode.txt\xFF\xFF\xFF"
              "print \" \"\xFF\xFF\xFF"
              "printh 03\xFF\xFF\xFF" );

  if( strlen( tx ) < 255 )
    nextion_send_buff( tx, strlen( tx ) );
}

uint8_t nextion_handle_req_config_txt( nextion_config *config )
{

  char rx[32];
  char * p;
  char * c;
  uint8_t i;
  uint8_t som = 0;
  uint8_t in;
  uint8_t v = 0;
  char var[4][8];
/*
  while( !mira_uart_receive_buffer_is_empty() )
  {
    in = mira_uart_receive_byte();

    if( som )
    {
      if( in == 0x03 )
      {
        rx[i] = '\0';
        p = rx;
        c = p;
        while( *p )
        {
          if( *p == ' ' )
          {
            *p = '\0';
            strcpy( var[v], c );
            v++;
            c = p + 1;
          }
          p++;
        }
        strcpy( (char*)config->SV, var[0] );
        strcpy( (char*)config->MaxOnTime, var[1] );
        strcpy( (char*)config->MaxOffTime, var[2] );
        strcpy( (char*)config->Mode, var[3] );
        return 1;
      }
      else
      {
        rx[i++] = in;
      }
    }

    if( in == 0x02 )
    {
      i = 0;
      som = 1;
    }
  }
*/
  return 0;

}

void nextion_task(void *pvParameter)
{
  nextion_init();

  while(1)
  {
    nextion_req_config_txt();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
