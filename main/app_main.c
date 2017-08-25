#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "mqtt.h"
#include "ds18b20.h"

#include "nextion.h"
#include "dbnode.h"

const char *MAIN_TAG = "DBNODE";

static TaskHandle_t xScanTask = NULL;
static TaskHandle_t xNextionTask = NULL;

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

node_runtime dbnode =
{
  .client = NULL,
  .blink_intervall = 200,
  .input.newflag = 0b00000001,
  .output.newflag = 0b00000011
};

// Function declaration

static void parse_mqtt_message( char *topic, char *payload);

void connected_cb(void *self, void *params)
{
  ESP_LOGI(MAIN_TAG, "[APP] Connected callback!");
  mqtt_client *client = (mqtt_client *)self;
  mqtt_subscribe(client, "/dbnode/+/set/#", 0);
  dbnode.blink_intervall = 1000;
}
void disconnected_cb(void *self, void *params)
{
  ESP_LOGI(MAIN_TAG, "[APP] Disconnected callback!");
  dbnode.blink_intervall = 500;
}
void subscribe_cb(void *self, void *params)
{
  ESP_LOGI(MAIN_TAG, "[APP] Subscribe callback!");
}
void publish_cb(void *self, void *params)
{
  ESP_LOGI(MAIN_TAG, "[APP] Publish callback!");
}
void data_cb(void *self, void *params)
{

  ESP_LOGI(MAIN_TAG, "[APP] Data callback!");
  mqtt_event_data_t *event_data = (mqtt_event_data_t *)params;

  if(event_data->data_offset == 0)
  {
    char *topic = malloc(event_data->topic_length + 1);
    memcpy(topic, event_data->topic, event_data->topic_length);
    topic[event_data->topic_length] = 0;
    ESP_LOGI(MAIN_TAG, "[APP] Publish topic: %s", topic);

    char *payload = malloc(event_data->data_length + 1);
    memcpy(payload, event_data->data, event_data->data_length);
    payload[event_data->data_length] = 0;

    parse_mqtt_message(topic, payload);

    free(payload);
    free(topic);
  }

  // char *data = malloc(event_data->data_length + 1);
  // memcpy(data, event_data->data, event_data->data_length);
  // data[event_data->data_length] = 0;
  ESP_LOGI(MAIN_TAG, "[APP] Publish data[%d/%d bytes]",
             event_data->data_length + event_data->data_offset,
             event_data->data_total_length);
  // data);

  // free(data);

}

static void parse_mqtt_message( char *topic, char *payload)
{
  char * token;

  token = strtok(topic, "/" );
  if(strcmp(token, "dbnode" ))
    return;

  token = strtok( NULL, "/" );
  if(strcmp(token, inet_ntoa(dbnode.ip_addr)))
    return;

  token = strtok( NULL, "/" );
  if(!strcmp(token, "set"))
  {
    if( payload[0] == '0' || payload[0] == '1')
    {
      token = strtok( NULL, "/" );
      switch (atoi(token))
      {
        case 32:
          gpio_set_level(GPIO_NUM_32, payload[0] == '1' ? 1 : 0);
        break;
        case 33:
          gpio_set_level(GPIO_NUM_33, payload[0] == '1' ? 1 : 0);
        break;
        default:
          ESP_LOGE("DBNODE","Undefined output request!");
      }
    }
  }
}

mqtt_settings settings = {
    .host = "172.19.2.39",
#if defined(CONFIG_MQTT_SECURITY_ON)
    .port = 8883, // encrypted
#else
    .port = 1883, // unencrypted
#endif
    //.client_id = "mqtt_client_id",
    .auto_reconnect = 1,
    .username = "user",
    .password = "pass",
    .clean_session = 0,
    .keepalive = 120,
    .lwt_topic = "/lwt",
    .lwt_msg = "offline",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = connected_cb,
    .disconnected_cb = disconnected_cb,
//    .reconnect_cb = reconnect_cb,
    .subscribe_cb = subscribe_cb,
    .publish_cb = publish_cb,
    .data_cb = data_cb
};

static void peripherial_init(void)
{
  gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(GPIO_NUM_33, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT);

}

void blink_task(void *pvParameter)
{
  int level = 1;
  while (true)
  {
    gpio_set_level(GPIO_NUM_16, level);
    level = !level;
    vTaskDelay( dbnode.blink_intervall / portTICK_PERIOD_MS);
  }
}

uint8_t get_current_inputs(void)
{
  uint8_t result = 0;
  uint8_t value;
  uint8_t c = 1;

  value = gpio_get_level(GPIO_NUM_34);
  result += value ? c : 0 ;
  c = c << 1;

  return result;
}

uint8_t get_current_outputs(void)
{
  uint8_t result = 0;
  uint8_t value;
  uint8_t c = 1;

  value = gpio_get_level(GPIO_NUM_32);
  result += value ? c : 0 ;
  c = c << 1;

  value = gpio_get_level(GPIO_NUM_33);
  result += value ? c : 0 ;
  c = c << 1;

  return result;
}

static const char *byte_to_binary(uint8_t x)
{
  static char b[9];
  b[0] = '\0';

  int z;
  for (z = 128; z > 0; z >>= 1)
  {
      strcat(b, ((x & z) == z) ? "1" : "0");
  }

  return b;
}

static void scan_task(void *pvParameter)
{
  char topic[64];
  uint8_t newflag;
  uint8_t c;

  while(true)
  {
    // Scan for changes
    dbnode.input.curflag = get_current_inputs();
    newflag = dbnode.input.curflag ^ dbnode.input.ackflag;
    if( newflag )
    {
      dbnode.input.newflag |= newflag;
    }
    dbnode.output.curflag = get_current_outputs();
    newflag = dbnode.output.curflag ^ dbnode.output.ackflag;
    if( newflag )
    {
      dbnode.output.newflag |= newflag;
    }
    // Get data to send but send only one change / cycle
    if(dbnode.input.newflag)
    {
      sprintf(topic,"/dbnode/%s/in/1",inet_ntoa(dbnode.ip_addr));
      mqtt_publish(dbnode.client, topic, byte_to_binary((uint8_t)dbnode.input.curflag), 8, 0, 0);
      // Clear ack- and newflag immediately, don't wait for ACK from server

      c = 0b00000001; // Mask for used bits
      dbnode.input.ackflag &= ~c;
      dbnode.input.ackflag |= dbnode.input.curflag & c;
      dbnode.input.newflag &= ~c;
    }
    else if(dbnode.output.newflag)
    {
      sprintf(topic,"/dbnode/%s/out/1",inet_ntoa(dbnode.ip_addr));
      mqtt_publish(dbnode.client, topic, byte_to_binary((uint8_t)dbnode.output.curflag), 8, 0, 0);

      c = 0b00000011; // Mask for used bits
      dbnode.output.ackflag &= ~c;
      dbnode.output.ackflag |= dbnode.output.curflag & c;
      dbnode.output.newflag &= ~c;
    }


    vTaskDelay( 50 / portTICK_PERIOD_MS);
  }
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            dbnode.blink_intervall = 500;
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            dbnode.ip_addr = event->event_info.got_ip.ip_info.ip;
            sprintf( settings.client_id,"dbnode-%s-%d", inet_ntoa(dbnode.ip_addr), (uint16_t)esp_random());
            dbnode.client = mqtt_start(&settings);
            xTaskCreate( &scan_task, "Scan", 2048, NULL, 5, &xScanTask );
            //init app here
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            dbnode.blink_intervall = 200; // no wifi!
            mqtt_stop();
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_conn_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(MAIN_TAG, "start the WIFI SSID:[%s] password:[%s]", CONFIG_WIFI_SSID, "******");
    ESP_ERROR_CHECK(esp_wifi_start());
}

void send_to_nextion_task( const char *var, const char* value )
{
  nextion_queue_message_t nextion_message;
  nextion_queue_message_t *pm;

  pm = &nextion_message;
  strcpy(pm->var, var);
  strcpy(pm->value,value);
  xQueueSend( xQueue_nextion, &pm, ( TickType_t ) 0 );
}

void app_main()
{
    ESP_LOGI(MAIN_TAG, "[APP] Startup..");

    nvs_flash_init();
    peripherial_init();
    xTaskCreate( &blink_task, "Blink", 2048, NULL, 5, NULL );
    inet_aton("127.0.0.1", &dbnode.ip_addr);
    wifi_conn_init();
    DS_init(17);
// Start nextion task here
    xTaskCreate( &nextion_task, "Nextion", 2048, NULL, 5, &xNextionTask );

#if 1
  char buffer[64];
  char temperature[5];
  while(1)
  {
    sprintf(temperature,"%3.1f", DS_get_temp());
    send_to_nextion_task("PV", temperature);
    vTaskDelay( 1000 / portTICK_PERIOD_MS);
  }
#else
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    char buffer[64];

    while(1)
    {
      nextion_req_config_txt();
      ESP_LOGI(MAIN_TAG,"Sent request!")
      vTaskDelay( 1000 / portTICK_PERIOD_MS);
      int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100 / portTICK_RATE_MS);
      if(len > 0)
      {
        ESP_LOGI(MAIN_TAG, "UART read : %d ", len);
      }
      else
      {
        ESP_LOGI(MAIN_TAG, "It's quiet");
      }

    }
#endif
}
