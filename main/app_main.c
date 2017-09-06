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
#include "modbus.h"

const char *MAIN_TAG = "DBNODE";

static TaskHandle_t xScanTask = NULL;
static TaskHandle_t xNextionTask = NULL;

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static config_node node;

runtime_node dbnode =
{
  .client = NULL,
  .blink_intervall = 200,
  .status.newflag = 0b00000001,
  .input.newflag  = 0b00000001,
  .output.newflag = 0b00000011
};

// == Private function prototypes ==============================

void send_to_nextion_task( nextion_queue_message_id_t id, const char *var, const char* value );
static void parse_mqtt_message( char *topic, char *payload);

// =============================================================

void connected_cb(void *self, void *params)
{
  ESP_LOGI(MAIN_TAG, "[APP] Connected callback!");
  mqtt_client *client = (mqtt_client *)self;
  mqtt_subscribe(client, "/dbnode/+/+/#", 0);
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
  uint16_t value;

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
  else if(!strcmp(token, "analog"))
  {
    ESP_LOGI(MAIN_TAG,"analog '%s'", payload);
    value = atoi(payload);
    if( value >= 0 && value <= 9999 )
    {
      token = strtok( NULL, "/" );
      switch (atoi(token))
      {
        case 1:
          node.pid.sv = value;
          ESP_LOGI(MAIN_TAG,"node.pid.sv = %d", node.pid.sv);
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

uint16_t DDdd_to_NNNN( char *val )
{
  char res[5];
  char *p;
  char dp;
  int  len;

  len = strlen( val );
  p = strchr( (char *)val, '.' );
  if( p != NULL )
  {
    dp = p - (char*)val; // Get position in string

    switch( dp )
    {
      case 1:
        res[0] = '0';
        res[1] = *( val + 0 );
        res[2] = *( val + 2 );
        if( len == 4 )
        {
          res[3] = *( val + 3 );
          res[4] = '\0';
        }
        else // len == 3
        {
          res[3] = '0';
          res[4] = '\0';
        }
        break;
      case 2:
        res[0] = *( val + 0 );
        res[1] = *( val + 1 );
        res[2] = *( val + 3 );
        res[3] = '0';
        res[4] = '\0';
        break;
    }
  }
  else
  {
    sprintf( res, "%s00", (char*)val );  // No decimal point
  }
  return atoi( res );
}

const char * NNNN_to_DDdd( uint16_t val )
{
  static char str[8];

  uint16_t i = val / 100;
  uint16_t d = ( val  - ( i * 100 ) ) / 10;
  sprintf( str,"%d.%d", i, d );

  return str;
}

uint8_t get_current_register_values()
{
  uint8_t result;
  uint8_t c;

  result = 0;
  c = 0b00000001; // Set Value

  if( display_config.SV[0] == '#' || !strlen(display_config.SV) )
  {
    ESP_LOGI(MAIN_TAG,"DEBUG: Override default ####/strlen=0 with %s", NNNN_to_DDdd( node.pid.sv ) );
    send_to_nextion_task(SET_STATUS_TEXT, "SV", NNNN_to_DDdd( node.pid.sv ) );
  }
  else
  {
    // Check if SV has been changed on the panel.
    if( DDdd_to_NNNN( display_config.SV ) != node.pid.sv )
    {
      //printf("display_config.SV = '%s' != node.pid.sv = %d \n", display_config.SV, node.pid.sv );
      node.pid.sv = DDdd_to_NNNN( display_config.SV );
    }
  }

  if( dbnode.analog.curval[REG_SV] != node.pid.sv )
  {
    ESP_LOGI(MAIN_TAG,"SET_STATUS_TEXT SV=%d",node.pid.sv);
    dbnode.analog.curval[REG_SV] = node.pid.sv;
    result += c;
    send_to_nextion_task(SET_STATUS_TEXT, "SV", NNNN_to_DDdd( node.pid.sv ) );
    //runtime.save_timeout = clock_seconds() + 10;
  }
/*
#endif
#ifdef DS2482
  c = 0b00000010; // Process Value
  if( iobox.reg.curval[REG_PV] != curtemp )
  {
    iobox.reg.curval[REG_PV] = curtemp;
    // Supress to fast update if value toggles up/down rapidly
    result += c;
#ifdef NEXTION
    COMSELECT_LCD;
    nextion_set_status_txt( "PV", NNNN_to_DDdd( curtemp ) );
#endif
  }
#endif

#ifdef THERMOSTAT
  c = 0b00000100; // Mode flag
#ifdef NEXTION
  switch( node.pid.mode )
  {
    case THERMOSTAT_MANUAL:
      sprintf( nextion_str,"Manual" );
      break;
    case THERMOSTAT_AUTO:
      sprintf( nextion_str,"OnOff" );
      break;
  }
  if( display_config.Mode[0] == '#' || !strlen(display_config.Mode) )
  {
    printf("DEBUG: Override default #### with %s\n", nextion_str );
    COMSELECT_LCD;
    nextion_set_config_txt( "Mode", nextion_str );
  }
  else
  {
    // Check if Mode has been changed on the panel.
    if( strcmp( display_config.Mode, nextion_str ) )
    {
      //printf("display_config.Mode = '%s' != node.pid.mode = '%s' \n", display_config.Mode, nextion_str );
      if( !strcmp( display_config.Mode, "Manual" ) )
      {
        node.pid.mode = THERMOSTAT_MANUAL;
        sprintf( nextion_str,"Manual" );               // Change nextion_str to reflect new value
      }
      if( !strcmp( display_config.Mode, "OnOff" ) )
      {
        node.pid.mode = THERMOSTAT_AUTO;
        sprintf( nextion_str,"OnOff" );
      }
    }
  }
#endif

  if( iobox.reg.curval[REG_MODE] != node.pid.mode )
  {
    iobox.reg.curval[REG_MODE] = node.pid.mode;
    result += c;
#ifdef NEXTION
    COMSELECT_LCD;
    nextion_set_status_txt( "Mode", nextion_str );
#endif
    runtime.save_timeout = clock_seconds() + 10;
  }


  c = 0b00001000; // MAX ON
#ifdef NEXTION
  sprintf( nextion_str,"%d", node.max_on );
  if( display_config.MaxOnTime[0] == '#' || !strlen(display_config.MaxOnTime) )
  {
    printf("DEBUG: Override default #### with %s\n", nextion_str );
    COMSELECT_LCD;
    nextion_set_config_txt( "MaxOnTime", nextion_str );
  }
  else
  {
    // Check if MaxOnTime has been changed on the panel.
    if( strcmp( display_config.MaxOnTime, nextion_str ) )
    {
      //printf("display_config.MaxOnTime = '%s' != node.max_on = '%s' \n", display_config.MaxOnTime, nextion_str );
      node.max_on = atoi( display_config.MaxOnTime );
      printf("atoi( display_config.MaxOnTime ) = %d\n", node.max_on);
    }
  }
#endif

  if( iobox.reg.curval[REG_MAX_ON] != node.max_on )
  {
    iobox.reg.curval[REG_MAX_ON] = node.max_on;
    result += c;
#ifdef NEXTION
    sprintf( nextion_str,"%d", node.max_on );
    COMSELECT_LCD;
    nextion_set_status_txt( "MaxOnTime", nextion_str );
#endif
    runtime.save_timeout = clock_seconds() + 10;
  }

  c = 0b00010000; // MAX OFF
#ifdef NEXTION
  sprintf( nextion_str,"%d", node.max_off );
  if( display_config.MaxOffTime[0] == '#' || !strlen(display_config.MaxOffTime) )
  {
    printf("DEBUG: Override default #### with %s\n", nextion_str );
    COMSELECT_LCD;
    nextion_set_config_txt( "MaxOffTime", nextion_str );
  }
  else
  {
    // Check if MaxOffTime has been changed on the panel.
    if( strcmp( display_config.MaxOffTime, nextion_str ) )
    {
      //printf("display_config.MaxOffTime = '%s' != node.max_off = '%s' \n", display_config.MaxOffTime, nextion_str );
      node.max_off = atoi( display_config.MaxOffTime );
      printf("atoi( display_config.MaxOffTime ) = %d\n", node.max_off);
    }
  }
#endif
  if( iobox.reg.curval[REG_MAX_OFF] != node.max_off )
  {
    iobox.reg.curval[REG_MAX_OFF] = node.max_off;
    result += c;
#ifdef NEXTION
    sprintf( nextion_str,"%d", node.max_off );
    COMSELECT_LCD;
    nextion_set_status_txt( "MaxOffTime", nextion_str );
#endif
  }

  c = 0b00100000; // Hysteresis PID output
  if( iobox.reg.curval[REG_HYST] != node.hysteresis )
  {
    iobox.reg.curval[REG_HYST] = node.hysteresis;
    result += c;
    runtime.save_timeout = clock_seconds() + 10;
  }
#endif
*/
/*
#ifdef MODBUS
  c = 0b00100000;
  if( iobox.reg.curval[REG_MODB_1] != modbus_register[0] )
  {
    iobox.reg.curval[REG_MODB_1] = modbus_register[0];
    result += c;
  }
  c = c << 1;
  if( iobox.reg.curval[REG_MODB_2] != modbus_register[1] )
  {
    iobox.reg.curval[REG_MODB_2] = modbus_register[1];
    result += c;
  }
  c = c << 1;
  if( iobox.reg.curval[REG_MODB_3] != modbus_register[2] )
  {
    iobox.reg.curval[REG_MODB_3] = modbus_register[2];
    result += c;
  }
#endif
*/
  return result;
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

void scan(void)
{
  uint8_t newflag;
  // input IO
  dbnode.input.curflag = get_current_inputs();
  newflag = dbnode.input.curflag ^ dbnode.input.ackflag;
  if( newflag )
  {
    dbnode.input.newflag |= newflag;
    return;
  }
  // output IO
  dbnode.output.curflag = get_current_outputs();
  newflag = dbnode.output.curflag ^ dbnode.output.ackflag;
  if( newflag )
  {
    dbnode.output.newflag |= newflag;
    return;
  }
  // status flags
  newflag = dbnode.status.curflag ^ dbnode.status.ackflag;
  if(newflag)
  {
    dbnode.status.newflag |= newflag;
    return;
  }
  // analog registers
  dbnode.analog.curflag = get_current_register_values();
  newflag = dbnode.analog.curflag ^ dbnode.analog.ackflag;
  if(newflag)
  {
    dbnode.analog.newflag |= newflag;
    return;
  }
}

static void scan_task(void *pvParameter)
{
  char topic[64];
  char value[32];
  uint8_t c;
  uint8_t i;

  while(true)
  {
    // Get data to send but send only one change / cycle
    scan();

    if(dbnode.input.newflag)
    {
      if( dbnode.client != NULL )
      {
        sprintf(topic,"/dbnode/%s/in/1",inet_ntoa(dbnode.ip_addr));
        mqtt_publish(dbnode.client, topic, byte_to_binary((uint8_t)dbnode.input.curflag), 8, 0, 0);
      }
      // Clear ack- and newflag immediately, don't wait for ACK from server
      c = 0b00000001; // Mask for used bits
      dbnode.input.ackflag &= ~c;
      dbnode.input.ackflag |= dbnode.input.curflag & c;
      dbnode.input.newflag &= ~c;
    }
    else if(dbnode.output.newflag)
    {
      if( dbnode.client != NULL )
      {
        sprintf(topic,"/dbnode/%s/out/1",inet_ntoa(dbnode.ip_addr));
        mqtt_publish(dbnode.client, topic, byte_to_binary((uint8_t)dbnode.output.curflag), 8, 0, 0);
      }
      c = 0b00000011; // Mask for used bits
      dbnode.output.ackflag &= ~c;
      dbnode.output.ackflag |= dbnode.output.curflag & c;
      dbnode.output.newflag &= ~c;
    }
    else if(dbnode.analog.newflag)
    {
      c = 1;
      for( i = 1; i <= 8; i++ )
      {
        if( dbnode.analog.newflag & c )
        {
          if( dbnode.client != NULL )
          {
            sprintf(value,"%d",dbnode.analog.curval[i-1]);
            sprintf(topic,"/dbnode/%s/analog/%d", inet_ntoa(dbnode.ip_addr), i);
            mqtt_publish(dbnode.client, topic, value, strlen(value), 0, 0);
          }
          dbnode.analog.ackflag &= ~c;
          dbnode.analog.ackflag |= dbnode.analog.curflag & c;
          dbnode.analog.newflag &= ~c;
        }
        c = c << 1;
      }
    }
    // Update status every cycle
    if(dbnode.status.newflag)
    {
      c = 1;
      for( i = 1; i <= 8; i++ )
      {
        if( dbnode.status.newflag & c )
        {
          switch(c)
          {
            case NETWORK_STATUS:
              send_to_nextion_task(SET_STATUS_TEXT, "Network", (dbnode.status.curflag & c) ? "Ok" : "Error" );
            break;
          }
          dbnode.status.ackflag &= ~c;
          dbnode.status.ackflag |= dbnode.status.curflag & c;
          dbnode.status.newflag &= ~c;
        }
        c = c << 1;
      }

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
            dbnode.status.curflag |= NETWORK_STATUS;
            //init app here
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            dbnode.blink_intervall = 200; // no wifi!
            dbnode.status.curflag &= ~NETWORK_STATUS;
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

void send_to_nextion_task( nextion_queue_message_id_t id, const char *var, const char* value )
{
  nextion_queue_message_t nextion_message;

  nextion_message.id = id;
  strcpy(nextion_message.var, var);
  strcpy(nextion_message.value,value);

  if(xQueue_nextion)
    xQueueSend( xQueue_nextion, &nextion_message, ( TickType_t ) 0 );

}

void app_main()
{
  char temperature[5];

  ESP_LOGI(MAIN_TAG, "[APP] Startup..");

  nvs_flash_init();
  peripherial_init();
  DS_init(17);

  xTaskCreate( &blink_task, "Blink", 2048, NULL, 5, NULL );
  xTaskCreate( &nextion_task, "Nextion", 2048, NULL, 5, &xNextionTask );
  xTaskCreate( &scan_task, "Scan", 2048, NULL, 5, &xScanTask );
  //xTaskCreate( &modbus_task, "Modbus", 2048, NULL, 5, NULL );

  inet_aton("127.0.0.1", &dbnode.ip_addr);
  wifi_conn_init();

  while(1)
  {
    sprintf(temperature,"%3.1f", DS_get_temp());
    send_to_nextion_task(SET_STATUS_TEXT, "PV", temperature);
    send_to_nextion_task(GET_CONFIG_TEXT, "", "");
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
