#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"

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
#include "mcp3551.h"

const char *MAIN_TAG = "DBNODE";

static TaskHandle_t xScanTask = NULL;
static TaskHandle_t xNextionTask = NULL;

static TimerHandle_t xTimerPID;

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static config_node node;

static runtime_node runtime;

// == Private function prototypes ==============================

void set_output( uint8_t iopin, uint8_t value );
esp_err_t save_config(void);
uint16_t get_temperature(void);
const char * NNNN_to_DDdd( uint16_t val );
void send_to_nextion_task( nextion_queue_message_id_t id, const char *var, const char* value );
static void parse_mqtt_message( char *topic, char *payload);

// =============================================================

void connected_cb(void *self, void *params)
{
  char topic[32];
  ESP_LOGI(MAIN_TAG, "[APP] Connected callback!");
  mqtt_client *client = (mqtt_client *)self;
  sprintf(topic,"/dbnode/%s/+/#", inet_ntoa(runtime.ip_addr));
  mqtt_subscribe(client, topic, 0);
  runtime.blink_intervall = 1000;
}
void disconnected_cb(void *self, void *params)
{
  ESP_LOGI(MAIN_TAG, "[APP] Disconnected callback!");
  runtime.blink_intervall = 500;
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

  //ESP_LOGI(MAIN_TAG, "[APP] Data callback!");
  mqtt_event_data_t *event_data = (mqtt_event_data_t *)params;

  if(event_data->data_offset == 0)
  {
    char *topic = malloc(event_data->topic_length + 1);
    memcpy(topic, event_data->topic, event_data->topic_length);
    topic[event_data->topic_length] = 0;

    char *payload = malloc(event_data->data_length + 1);
    memcpy(payload, event_data->data, event_data->data_length);
    payload[event_data->data_length] = 0;

    ESP_LOGI(MAIN_TAG, "[APP] MQTT topic received: %s %s", topic, payload);

    parse_mqtt_message(topic, payload);

    free(payload);
    free(topic);
  }

}

static void parse_mqtt_message( char *topic, char *payload)
{
  char * token;
  uint16_t value;

  token = strtok(topic, "/" );
  if(strcmp(token, "dbnode" ))
    return;

  token = strtok( NULL, "/" );
  if(strcmp(token, inet_ntoa(runtime.ip_addr)))
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
          set_output(GPIO_NUM_32, payload[0] == '1' ? 1 : 0);
          //gpio_set_level(GPIO_NUM_32, payload[0] == '1' ? 1 : 0);
        break;
        case 33:
          set_output(GPIO_NUM_33, payload[0] == '1' ? 1 : 0);
          //gpio_set_level(GPIO_NUM_33, payload[0] == '1' ? 1 : 0);
        break;
        default:
          ESP_LOGE("DBNODE","Undefined output request!");
      }
    }
  }
  else if(!strcmp(token, "setreg"))
  {
    char valstr[16];

    value = atoi(payload);
    if( value <= 9999 )
    {
      token = strtok( NULL, "/" );
      ESP_LOGI(MAIN_TAG,"/dbnode/%s/setreg/%s '%s'", inet_ntoa(runtime.ip_addr), token, payload);
      switch(atoi(token))
      {
        case 1: // Set value
          node.pid.sv = value; // Is this necessary to set here?
          send_to_nextion_task(SET_CONFIG_TEXT, "SV", NNNN_to_DDdd( value ) );
        break;
        case 2: // Process value ( read only )
        break;
        case 3: // PID Mode
          switch(value)
          {
            case 0:
              send_to_nextion_task(SET_CONFIG_TEXT, "Mode", "Manual" );
            break;
            case 1:
              send_to_nextion_task(SET_CONFIG_TEXT, "Mode", "OnOff" );
            break;
          }
        break;
        case 4: // MaxOnTime -> Pulse width PWM output
          if(value > 0 && value < node.output_cycle_time)
          {
            node.output_pulse_width = value;
            send_to_nextion_task(SET_CONFIG_TEXT, "MaxOnTime", itoa(value, valstr, 10));
          }
        break;
        case 5: // MaxOffTime -> Cycle time PWM output
          if(value > 0)
          {
            node.output_cycle_time = value;
            send_to_nextion_task(SET_CONFIG_TEXT, "MaxOffTime", itoa(value, valstr, 10));
          }
        break;
        case 6: // Hysteresis
          if(value >= 0 && value <= (node.pid.out_max / 2))
            node.hysteresis = value;
          //send_to_nextion_task(SET_CONFIG_TEXT, "Hysteresis", itoa(value, valstr, 10));
        break;
        default:
          ESP_LOGE("DBNODE","Undefined output request!");
      }
    }
  }
}

mqtt_settings settings = {
//  .host = "172.19.2.39",
    .host = "10.10.1.195",
#if defined(CONFIG_MQTT_SECURITY_ON)
    .port = 8883, // encrypted
#else
    .port = 1883, // unencrypted
#endif
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
    vTaskDelay( runtime.blink_intervall / portTICK_PERIOD_MS);
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
  uint8_t   result;
  uint8_t   c;
  char      nextion_str[16];
  uint16_t  curtemp;

  result = 0;
  c = 0b00000001; // Set Value

  if( display_config.SV[0] == '#' || !strlen(display_config.SV) )
  {
    //ESP_LOGI(MAIN_TAG,"DEBUG: Override default '%s' with '%s'", display_config.SV, NNNN_to_DDdd( node.pid.sv ) );
    send_to_nextion_task(SET_CONFIG_TEXT, "SV", NNNN_to_DDdd( node.pid.sv ) );
  }
  else
  {
    // Check if SV has been changed on the panel.
    if( DDdd_to_NNNN( display_config.SV ) != node.pid.sv )
    {
      node.pid.sv = DDdd_to_NNNN( display_config.SV );
    }
  }

  if( runtime.reg16.curval[REG_SV] != node.pid.sv )
  {
    runtime.reg16.curval[REG_SV] = node.pid.sv;
    result += c;
    send_to_nextion_task(SET_STATUS_TEXT, "SV", NNNN_to_DDdd( node.pid.sv ));
    save_config();
    //runtime.save_timeout = clock_seconds() + 10;
  }

  c = 0b00000010; // Process Value
  curtemp = get_temperature();
  if( curtemp < 9999 )
  {
    if( runtime.reg16.curval[REG_PV] != curtemp )
    {
      runtime.reg16.curval[REG_PV] = curtemp;
      // Supress to fast update if value toggles up/down rapidly
      result += c;
      send_to_nextion_task(SET_STATUS_TEXT, "PV", NNNN_to_DDdd( curtemp ) );
    }
  }

  c = 0b00000100; // Mode flag
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
    send_to_nextion_task(SET_CONFIG_TEXT, "Mode", nextion_str );
  }
  else
  {
    // Check if Mode has been changed on the panel.
    if( strcmp( display_config.Mode, nextion_str ) )
    {
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

  if( runtime.reg16.curval[REG_MODE] != node.pid.mode )
  {
    runtime.reg16.curval[REG_MODE] = node.pid.mode;
    result += c;
    send_to_nextion_task(SET_STATUS_TEXT, "Mode", nextion_str );
    save_config();
    //runtime.save_timeout = clock_seconds() + 10;
  }
//=============================================================================

  c = 0b00001000; // MAX ON
  sprintf( nextion_str,"%d", node.output_pulse_width );
  if( display_config.MaxOnTime[0] == '#' || !strlen(display_config.MaxOnTime) )
  { // Nextion coldstart values
    send_to_nextion_task(SET_CONFIG_TEXT, "MaxOnTime", nextion_str );
  }
  else
  {
    // Update node.output_pulse_width if MaxOnTime has been changed on the panel.
    if( strcmp( display_config.MaxOnTime, nextion_str ) )
    {
      node.output_pulse_width = atoi( display_config.MaxOnTime );
    }
  }

  if( runtime.reg16.curval[REG_MAX_ON] != node.output_pulse_width )
  {
    runtime.reg16.curval[REG_MAX_ON] = node.output_pulse_width;
    result += c;
    sprintf( nextion_str,"%d", node.output_pulse_width );
    send_to_nextion_task(SET_STATUS_TEXT, "MaxOnTime", nextion_str );
    save_config();
    //runtime.save_timeout = clock_seconds() + 10;
  }

  //=============================================================================

  c = 0b00010000; // MAX OFF
  sprintf( nextion_str,"%d", node.output_cycle_time );
  if( display_config.MaxOffTime[0] == '#' || !strlen(display_config.MaxOffTime) )
  { // Nextion coldstart values
    send_to_nextion_task(SET_CONFIG_TEXT, "MaxOffTime", nextion_str );
  }
  else
  {
    if( strcmp( display_config.MaxOffTime, nextion_str ) )
    {
      node.output_cycle_time = atoi( display_config.MaxOffTime );
    }
  }

  if( runtime.reg16.curval[REG_MAX_OFF] != node.output_cycle_time )
  {
    runtime.reg16.curval[REG_MAX_OFF] = node.output_cycle_time;
    result += c;
    sprintf( nextion_str,"%d", node.output_cycle_time );
    send_to_nextion_task(SET_STATUS_TEXT, "MaxOffTime", nextion_str );
    save_config();
    //runtime.save_timeout = clock_seconds() + 10;
  }

  c = 0b00100000; // Hysteresis PID output
  if( runtime.reg16.curval[REG_HYST] != node.hysteresis )
  {
    runtime.reg16.curval[REG_HYST] = node.hysteresis;
    result += c;
    save_config();
    //runtime.save_timeout = clock_seconds() + 10;
  }
/*
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

void set_output( uint8_t iopin, uint8_t value )
{

  switch(iopin)
  {
    case GPIO_NUM_32:
      send_to_nextion_task(SET_STATUS_TEXT, "Output1", value == 1 ? "On" : "Off" );
    break;
    case GPIO_NUM_33:
      send_to_nextion_task(SET_STATUS_TEXT, "Output2", value == 1 ? "On" : "Off" );
    break;
    default:
      return;
  }

  if(value == 1)
  {
    gpio_set_level(iopin, 1);
  }
  else if(value == 0)
  {
    gpio_set_level(iopin, 0);
  }
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
  runtime.input.curflag = get_current_inputs();
  newflag = runtime.input.curflag ^ runtime.input.ackflag;
  if( newflag )
  {
    runtime.input.newflag |= newflag;
    return;
  }
  // output IO
  runtime.output.curflag = get_current_outputs();
  newflag = runtime.output.curflag ^ runtime.output.ackflag;
  if( newflag )
  {
    runtime.output.newflag |= newflag;
    return;
  }
  // status flags
  newflag = runtime.status.curflag ^ runtime.status.ackflag;
  if(newflag)
  {
    runtime.status.newflag |= newflag;
    return;
  }
  // 16-bit registers
  runtime.reg16.curflag = get_current_register_values();
  newflag = runtime.reg16.curflag ^ runtime.reg16.ackflag;
  if(newflag)
  {
    runtime.reg16.newflag |= newflag;
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

    if(runtime.input.newflag)
    {
      if( runtime.client_mqtt != NULL )
      {
        sprintf(topic,"/dbnode/%s/in/1",inet_ntoa(runtime.ip_addr));
        mqtt_publish(runtime.client_mqtt, topic, byte_to_binary((uint8_t)runtime.input.curflag), 8, 0, 0);
      }
      // Clear ack- and newflag immediately, don't wait for ACK from server
      c = 0b00000001; // Mask for used bits
      runtime.input.ackflag &= ~c;
      runtime.input.ackflag |= runtime.input.curflag & c;
      runtime.input.newflag &= ~c;
    }
    else if(runtime.output.newflag)
    {
      if( runtime.client_mqtt != NULL )
      {
        sprintf(topic,"/dbnode/%s/out/1",inet_ntoa(runtime.ip_addr));
        mqtt_publish(runtime.client_mqtt, topic, byte_to_binary((uint8_t)runtime.output.curflag), 8, 0, 0);
      }
      c = 0b00000011; // Mask for used bits
      runtime.output.ackflag &= ~c;
      runtime.output.ackflag |= runtime.output.curflag & c;
      runtime.output.newflag &= ~c;
    }
    else if(runtime.reg16.newflag)
    {
      c = 1;
      for( i = 1; i <= 8; i++ )
      {
        if( runtime.reg16.newflag & c )
        {
          if( runtime.client_mqtt != NULL )
          {
            sprintf(value,"%d",runtime.reg16.curval[i-1]);
            sprintf(topic,"/dbnode/%s/register/%d", inet_ntoa(runtime.ip_addr), i);
            mqtt_publish(runtime.client_mqtt, topic, value, strlen(value), 0, 0);
          }
          // Clear newflag only to prevent double trig.
          runtime.reg16.newflag &= ~c;
        }
        c = c << 1;
      }
    }
    // Update status every cycle
    if(runtime.status.newflag)
    {
      c = 1;
      for( i = 1; i <= 8; i++ )
      {
        if( runtime.status.newflag & c )
        {
          switch(c)
          {
            case NETWORK_STATUS:
              send_to_nextion_task(SET_STATUS_TEXT, "Network", (runtime.status.curflag & c) ? "Ok" : "Error" );
            break;
          }
          runtime.status.ackflag &= ~c;
          runtime.status.ackflag |= runtime.status.curflag & c;
          runtime.status.newflag &= ~c;
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
            runtime.blink_intervall = 500;
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            runtime.ip_addr = event->event_info.got_ip.ip_info.ip;
            sprintf( settings.client_id,"dbnode-%s-%d", inet_ntoa(runtime.ip_addr), (uint16_t)esp_random());
            runtime.client_mqtt = mqtt_start(&settings);
            runtime.status.curflag |= NETWORK_STATUS;
            //init app here
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            runtime.blink_intervall = 200; // no wifi!
            runtime.status.curflag &= ~NETWORK_STATUS;
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

uint16_t get_temperature(void)
{
  uint16_t td;
  float tf;
  char ts[8];

  tf = DS_get_temp();
  sprintf( ts,"%3.1f",tf);
  td = tf * 10;
  td *= 10;

  //======= PT100

//  float factor = (42870 - 31140)/100;  // units per C
//  tf = (float)(mcp3551_value - 31140) / factor;
  float factor = 6409;  // units per C
  tf = (float)((mcp3551_value / factor) - 60.0);
  td = (uint16_t)(tf*10);
  td *= 10;
  //ESP_LOGI(MAIN_TAG,"mcp3551_value = %d tf = %f td = %d", mcp3551_value, tf, td);

  return td;
}

void clear_runtime(void)
{
  inet_aton("127.0.0.1", &runtime.ip_addr);
  runtime.client_mqtt = NULL;
  runtime.client_mqtt = NULL;
  runtime.blink_intervall = 200;
  runtime.status.newflag = 0b00000001;
  runtime.input.newflag  = 0b00000001;
  runtime.output.newflag = 0b00000011;
}

esp_err_t read_config(void)
{
  nvs_handle nvh;
  esp_err_t err;


  // Default values
  node.mode = COLD;
  node.output_pulse_width = 120;
  node.output_cycle_time = 300;
  node.hysteresis = 10;

  node.pid.mode = THERMOSTAT_AUTO;
  node.pid.sv = 1800;
  node.pid.kp = -1;
  node.pid.ki = 0;
  node.pid.kd = 0;
  node.pid.sample_time = 2;
  node.pid.out_max = 200;
  node.pid.out_min = -200;


  err = nvs_open("dbnode", NVS_READWRITE, &nvh);
  if(err != ESP_OK)
    return err;

  nvs_get_u16(nvh, "mode", &node.mode);
  nvs_get_u16(nvh, "max_on", &node.output_pulse_width);
  nvs_get_u16(nvh, "max_off", &node.output_cycle_time);
  nvs_get_u16(nvh, "hysteresis", &node.hysteresis);

  nvs_get_u8(nvh, "pid.mode", &node.pid.mode);
  nvs_get_u8(nvh, "pid.direction", &node.pid.direction);
  nvs_get_i16(nvh, "pid.sv", &node.pid.sv);
  nvs_get_i16(nvh, "pid.kp", &node.pid.kp);
  nvs_get_i16(nvh, "pid.ki", &node.pid.ki);
  nvs_get_i16(nvh, "pid.kd", &node.pid.kd);
  nvs_get_i16(nvh, "pid.out_max", &node.pid.out_max);
  nvs_get_i16(nvh, "pid.out_min", &node.pid.out_min);
  nvs_get_i16(nvh, "pid.sample_time", &node.pid.sample_time);

  return ESP_OK;
}

esp_err_t save_config(void)
{
  nvs_handle nvh;
  esp_err_t err;

  ESP_LOGI(MAIN_TAG,"save_config()");

  err = nvs_open("dbnode", NVS_READWRITE, &nvh);
  if(err != ESP_OK)
    return err;

  nvs_set_u16(nvh, "mode", node.mode);
  nvs_set_u16(nvh, "max_on", node.output_pulse_width);
  nvs_set_u16(nvh, "max_off", node.output_cycle_time);
  nvs_set_u16(nvh, "hysteresis", node.hysteresis);

  nvs_set_u8(nvh, "pid.mode", node.pid.mode);
  nvs_set_u8(nvh, "pid.direction", node.pid.direction);
  nvs_set_i16(nvh, "pid.sv", node.pid.sv);
  nvs_set_i16(nvh, "pid.kp", node.pid.kp);
  nvs_set_i16(nvh, "pid.ki", node.pid.ki);
  nvs_set_i16(nvh, "pid.kd", node.pid.kd);
  nvs_set_i16(nvh, "pid.out_max", node.pid.out_max);
  nvs_set_i16(nvh, "pid.out_min", node.pid.out_min);
  nvs_set_i16(nvh, "pid.sample_time", node.pid.sample_time);

  err = nvs_commit(nvh);
  if(err != ESP_OK)
    return err;

  nvs_close(nvh);
  return ESP_OK;
}

void pid_timer_cb( TimerHandle_t xTimer )
{
  static uint8_t outflag = 0;
  unsigned long now = xTaskGetTickCount() / 1000;

  //ESP_LOGI( MAIN_TAG,"xTaskGetTickCount() / 1000 = %ld", now);
  pid_compute( &node.pid, get_temperature() );

/*
    ____________________                                  __________
___|                    |________________________________|

    |--------------------  Cycle time --------------------| Former MaxOffTime

    |---- Pulse width ---|  Former MaxOnTime

*/

  if( ( node.pid.output > node.hysteresis ) && ( node.pid.mode == THERMOSTAT_AUTO ) )
  {
    if( outflag == 0 ) // if outflag == 0 then we're going off->on
    {
      outflag = 1;
      send_to_nextion_task(SET_STATUS_TEXT, "PIDout", "1" );
    }

    if( runtime.off_timeout <= now )
    {
//      runtime.off_timeout = now + node.output_pulse_width + node.output_cycle_time;
      runtime.on_timeout = now + node.output_pulse_width;
      runtime.off_timeout = now + node.output_cycle_time;
      set_output(GPIO_NUM_32, 1 );
      set_output(GPIO_NUM_33, 1 );
      ESP_LOGI( MAIN_TAG, "%ld : On -> On tmo %d (Off tmo %d)", now, runtime.on_timeout, runtime.off_timeout );
    }

    if( runtime.on_timeout <= now )
    {
//      runtime.off_timeout = now + node.output_cycle_time;
//      runtime.on_timeout = now + node.output_cycle_time + node.output_pulse_width;
      runtime.on_timeout = now + node.output_pulse_width;
      set_output(GPIO_NUM_32, 0 );
      set_output(GPIO_NUM_33, 0 );
      ESP_LOGI( MAIN_TAG, "%ld : Off -> Off tmo %d (On tmo %d)", now, runtime.off_timeout, runtime.on_timeout );
    }
  }
  else if( ( node.pid.output < -node.hysteresis ) || ( node.pid.mode != THERMOSTAT_AUTO ) )
  {
    if( outflag ) // if outflag != 0 then we're going on->off
    {
      outflag = 0;
      send_to_nextion_task(SET_STATUS_TEXT, "PIDout", "0" );
      set_output(GPIO_NUM_32, 0 );
      set_output(GPIO_NUM_33, 0 );
      runtime.on_timeout = 0;
      runtime.off_timeout = 0;
    }
  }
}

void app_main()
{

  ESP_LOGI(MAIN_TAG, "[APP] Startup..");

  nvs_flash_init();
  peripherial_init();
  DS_init(17);

  clear_runtime();
  read_config();

  ESP_LOGI(MAIN_TAG,"node.pid.sv = %d", node.pid.sv);

  if(node.mode == COLD)
  {
    node.mode = RUN;
    save_config();
  }

  xTaskCreate( &blink_task, "Blink", 2048, NULL, 5, NULL );
  xTaskCreate( &nextion_task, "Nextion", 2048, NULL, 5, &xNextionTask );
  xTaskCreate( &scan_task, "Scan", 2048, NULL, 5, &xScanTask );
  //xTaskCreate( &modbus_task, "Modbus", 2048, NULL, 5, NULL );
  xTaskCreate( &mcp3551_task, "MCP3551", 2048, NULL, 5, NULL );

  wifi_conn_init();

  // Timer for PID regulator.
  xTimerPID = xTimerCreate("PID", (1000*node.pid.sample_time)/portTICK_PERIOD_MS, pdTRUE, ( void * ) 0, pid_timer_cb );
  pid_init(&node.pid, get_temperature());
  xTimerStart(xTimerPID, 0);

  while(1)
  {
    send_to_nextion_task(GET_CONFIG_TEXT, "", "");
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
