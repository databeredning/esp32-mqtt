#include <stdio.h>
#include "modbus.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "esp_log.h"

#define BUF_SIZE (1024)
#define BUFFER_SIZE 128

const char *MODBUS_TAG = "MODBUS";
static QueueHandle_t uart_queue;
static uint8_t frame[BUFFER_SIZE];

static uint16_t calculateCRC(uint8_t bufferSize);
static uint8_t  getresponse();

uint8_t preset_single_register( uint8_t id, uint16_t address, uint16_t value )
{
  unsigned int crc16;

  frame[0] = id;             // ID
  frame[1] = 0x06;           // Function #
  frame[2] = address >> 8;   // Address Hi
  frame[3] = address & 0xFF; // Address Lo
  frame[4] = value >> 8;     // Value of registers Hi
  frame[5] = value & 0xFF;   // Value of register Lo
  crc16 = calculateCRC( 6 );
  frame[6] = crc16 >> 8;     // crc Hi
  frame[7] = crc16 & 0xFF;   // crc Lo

  uart_write_bytes( UART_NUM_2, (char*)frame, 8 );
  ESP_LOGI(MODBUS_TAG, "FC #06 Preset single register");

  return 0;
}

uint8_t read_holding_registers( uint8_t id, uint16_t address, uint16_t numreg )
{
  unsigned int crc16;

  frame[0] = id;             // ID
  frame[1] = 0x03;           // Function #
  frame[2] = address >> 8;   // Address Hi
  frame[3] = address & 0xFF; // Address Lo
  frame[4] = numreg >> 8;    // Number of registers Hi
  frame[5] = numreg & 0xFF;  // Number of register Lo
  crc16 = calculateCRC( 6 );
  frame[6] = crc16 >> 8;     // crc Hi
  frame[7] = crc16 & 0xFF;   // crc Lo

  uart_write_bytes( UART_NUM_2, (char*)frame, 8 );
  ESP_LOGI(MODBUS_TAG, "FC #03 Read holding registers");

  return 0;
}

static uint8_t getresponse()
{
  uint8_t len;
  uint16_t crc16;

  len = uart_read_bytes(UART_NUM_2, (uint8_t*)frame, BUFFER_SIZE, 100 / portTICK_RATE_MS);

  switch(frame[1])
  {
    case 0x03:
      crc16 = calculateCRC( 3 + frame[2] ); // id,func,len + len
    break;
    case 0x06:
      crc16 = calculateCRC( 6 ); // Always len 6
    break;
    default:
      return 0;
  }

  ESP_LOGI(MODBUS_TAG,"CRC in = %01X%01X ut = %02X", frame[len-2],frame[len-1], crc16)

  if( ( crc16 >> 8 ) == frame[len - 2] && ( crc16 & 0xFF ) == frame[len - 1]  )
    return 1;
  else
    return 0;

}

static uint16_t calculateCRC(uint8_t bufferSize)
{
  uint16_t temp, temp2, flag;
  uint8_t i, j;

  temp = 0xFFFF;
  for( i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for( j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order.
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  return temp; // the returned value is already swopped - crcLo byte is first & crcHi byte is last
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);

    ESP_LOGI(MODBUS_TAG, "UART 2 event task started.");

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.
                in this example, we don't process data in event, but read data outside.*/
                case UART_DATA:
                    uart_get_buffered_data_len(UART_NUM_2, &buffered_size);
                    ESP_LOGI(MODBUS_TAG, "data, len: %d; buffered len: %d", event.size, buffered_size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(MODBUS_TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(UART_NUM_2);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(MODBUS_TAG, "ring buffer full\n");
                    //If buffer full happened, you should consider encreasing your buffer size
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(UART_NUM_2);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(MODBUS_TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(MODBUS_TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(MODBUS_TAG, "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(MODBUS_TAG, "uart pattern detected\n");
                    break;
                //Others
                default:
                    ESP_LOGI(MODBUS_TAG, "uart event type: %d\n", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
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
  uart_driver_install(uart, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, 0);
  uart_set_pin(uart, txpin, rxpin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  //Create a task to handle UART event from ISR
  xTaskCreate(uart_event_task, "uart2_event_task", 2048, NULL, 12, NULL);
}

void modbus_task(void *pvParameter)
{
  static uint16_t counter = 0;
  ESP_LOGI(MODBUS_TAG, "Modbus task started.");
  uart_init(UART_NUM_2, GPIO_NUM_10, GPIO_NUM_9);

  vTaskDelay(5000/portTICK_PERIOD_MS);

  while( 1 )
  {

    preset_single_register(0x05, 0x01, counter++);
    vTaskDelay(100/portTICK_PERIOD_MS);

    if( getresponse() )
    {
      ESP_LOGI(MODBUS_TAG,"Preset ok!");
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);

    read_holding_registers(0x05, 0x00, 0x03 );
    vTaskDelay(100/portTICK_PERIOD_MS);

    if( getresponse() )
    {
      modbus_register[0] = (uint16_t)frame[3] << 8 | (uint16_t)frame[4];
      modbus_register[1] = (uint16_t)frame[5] << 8 | (uint16_t)frame[6];
      modbus_register[2] = (uint16_t)frame[7] << 8 | (uint16_t)frame[8];
      ESP_LOGI(MODBUS_TAG,"Register read: %02X %02X %02X", modbus_register[0], modbus_register[1], modbus_register[2]);
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);

  }
}
