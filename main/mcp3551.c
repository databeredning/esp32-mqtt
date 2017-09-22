#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <driver/spi_master.h>
#include "mcp3551.h"

static char tag[] = "SPI";

void mcp3551_task(void *pvParameter)
{

	spi_bus_config_t bus_config;
  memset(&bus_config,0,sizeof(bus_config));

	bus_config.sclk_io_num = 18;   // CLK
	bus_config.mosi_io_num = -1;   // MOSI
	bus_config.miso_io_num = 19;   // MISO
	bus_config.quadwp_io_num = -1; // Not used
	bus_config.quadhd_io_num = -1; // Not used

	spi_device_handle_t handle;
	spi_device_interface_config_t dev_config;
  memset(&dev_config,0,sizeof(dev_config));

	dev_config.address_bits = 0;
	dev_config.command_bits = 0;
	dev_config.dummy_bits = 0;
	dev_config.mode = 3; // SPI Mode 3
	dev_config.duty_cycle_pos = 0;
	dev_config.cs_ena_posttrans = 0;
	dev_config.cs_ena_pretrans = 0;
	dev_config.clock_speed_hz = 10000;
	dev_config.spics_io_num = -1;//5; // SS /CS
	dev_config.flags = SPI_DEVICE_HALFDUPLEX;
	dev_config.queue_size = 1;
	dev_config.pre_cb = NULL;
	dev_config.post_cb = NULL;

  char rx_data[3];

  union
  {
    char    buf[4];
    int32_t value;
  } ad;

	spi_transaction_t trans_desc;
  memset(&trans_desc,0,sizeof(trans_desc));

	trans_desc.address = 0;
	trans_desc.command = 0;
	trans_desc.flags = 0; //SPI_TRANS_USE_RXDATA;// | SPI_TRANS_USE_TXDATA;
	trans_desc.length = 0 * 8;
	trans_desc.rxlength = 3 * 8;
	trans_desc.tx_buffer = NULL;
	trans_desc.rx_buffer = rx_data;

  // /CS high
  gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_5,1);

  ESP_LOGI(tag, "... Initializing bus.");
  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));

  ESP_LOGI(tag, "... Adding device bus.");
  ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &handle));

  while(true)
  {
    gpio_set_level(GPIO_NUM_5,0);
    vTaskDelay( 75 / portTICK_PERIOD_MS); // tconv = 72.73 mS

  	ESP_ERROR_CHECK(spi_device_transmit(handle, &trans_desc));

    ad.buf[0] = rx_data[2];
    ad.buf[1] = rx_data[1];
    ad.buf[2] = rx_data[0];
    ad.buf[3] = 0x00;
/*
    if((rx_data[2]&(1<<6))|(rx_data[2]&(1<<7)))
    {
      rx_data[2]&=~(1<<6);
    }
    //check if sign bit is affected. if so, since it is two's compliment,
    // substract it from 2^N
    else if(rx_data[2]&(1<<5))
    {
      ad.value=0x400000-ad.value;
    }
*/
    //ESP_LOGI(tag, "%02X %02X %02X = %d", rx_data[0], rx_data[1], rx_data[2], ad.value);

    mcp3551_value = ad.value;

    vTaskDelay( 5 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_5,1);
    vTaskDelay( 4920 / portTICK_PERIOD_MS);

  }
  // Will never reach here
  ESP_ERROR_CHECK(spi_bus_remove_device(handle));
  ESP_ERROR_CHECK(spi_bus_free(HSPI_HOST));
}
