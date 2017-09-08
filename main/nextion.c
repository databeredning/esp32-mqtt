#include <stdio.h>
#include <string.h>
#include "nextion.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "esp_log.h"

const char *NEXTION_TAG = "NEXTION";
static QueueHandle_t uart1_queue;
QueueHandle_t xQueue_nextion;

nextion_config display_config;
char rx[64];

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

void nextion_get_config_txt( const char *reg )
{
  char tx[32];

  sprintf( tx, "print config.%s.txt\xFF\xFF\xFF", reg );
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

uint8_t nextion_handle_uart_data_event(uint8_t buffered_size)
{
  uint8_t len;
  static uint8_t som = 0;
  static uint8_t ri = 0;
  uint8_t data[64];
  char * p;
  char * c;
  uint8_t v = 0;
  char var[4][8];

  len = uart_read_bytes(UART_NUM_1, data, sizeof(data), 100 / portTICK_RATE_MS);

  for(int i = 0; i < len; i++)
  {
    if( som )
    {
      if( data[i] == 0x03 )
      {
        som = 0;
        rx[ri] = '\0';
        //ESP_LOGI(NEXTION_TAG,"rx = '%s'", rx);

        p = rx;
        c = p;
        while( *p )
        {
          if( *p == ' ' )
          {
            *p = '\0';
            strcpy(var[v], c );
            v++;
            c = p + 1;
          }
          p++;
        }
        strcpy((char*)display_config.SV, var[0]);
        strcpy((char*)display_config.MaxOnTime, var[1]);
        strcpy((char*)display_config.MaxOffTime, var[2]);
        strcpy((char*)display_config.Mode, var[3]);

        return 1;
      }
      else
      {
        rx[ri++] = data[i];
      }
    }
    else if( data[i] == 0x02 )
    {
      som = 1;
      ri = 0;
    }
    else
    {
      //ESP_LOGI(NEXTION_TAG,"UART data[%d] = %02X", i, data[i]);
    }

  }

  return 0;
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;

    ESP_LOGI(NEXTION_TAG, "UART 1 event task started.");

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.
                in this example, we don't process data in event, but read data outside.*/
                case UART_DATA:
                    uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
//                    ESP_LOGI(NEXTION_TAG, "UART_NUM_1 data len: %d; buffered len: %d", event.size, buffered_size);
//                    if(event.size > 0)
                    if(buffered_size> 0)
                      nextion_handle_uart_data_event(buffered_size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(NEXTION_TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(UART_NUM_1);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(NEXTION_TAG, "ring buffer full\n");
                    //If buffer full happened, you should consider encreasing your buffer size
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(UART_NUM_1);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(NEXTION_TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(NEXTION_TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(NEXTION_TAG, "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(NEXTION_TAG, "uart pattern detected\n");
                    break;
                //Others
                default:
                    ESP_LOGI(NEXTION_TAG, "uart event type: %d\n", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void uart_init( uart_port_t uart, gpio_num_t txpin, gpio_num_t rxpin )
{
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
  };

  uart_param_config(uart, &uart_config);
  uart_driver_install(uart, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart1_queue, 0);
  uart_set_pin(uart, txpin, rxpin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  //Create a task to handle UART event from ISR
  xTaskCreate(uart_event_task, "uart1_event_task", 2048, NULL, 12, NULL);
}

void nextion_task(void *pvParameter)
{
  nextion_queue_message_t nextion_message;

  ESP_LOGI(NEXTION_TAG, "Nextion task started.");
  //uart_init(UART_NUM_1, GPIO_NUM_10, GPIO_NUM_9);
  uart_init(UART_NUM_1, GPIO_NUM_27, GPIO_NUM_26);
  nextion_init();

  xQueue_nextion = xQueueCreate( 10, sizeof( nextion_queue_message_t ) );
  if( xQueue_nextion == 0 )
  {
    ESP_LOGE(NEXTION_TAG,"Error creating QueueHandle_t in %s", __FILE__ );
  }

  // Make sure init finishes before we go on.
  vTaskDelay(50 / portTICK_PERIOD_MS);

  while(1)
  {
    if(xQueueReceive(xQueue_nextion, &nextion_message, (portTickType)portMAX_DELAY))
    {
      //ESP_LOGI(NEXTION_TAG,"Message received! %d", nextion_message.id);
      switch(nextion_message.id)
      {
        case SET_STATUS_TEXT:
          nextion_set_status_txt(nextion_message.var, nextion_message.value);
        break;
        case SET_CONFIG_TEXT:
          nextion_set_config_txt(nextion_message.var, nextion_message.value);
        break;
        case GET_CONFIG_TEXT:
          nextion_req_config_txt();
        break;
        case GET_STATUS_TEXT:
          // Not implemneted yet
        break;
      }
    }
  }
  // Should never get here
  vTaskDelete(NULL);
}
