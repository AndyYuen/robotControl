/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include<string.h>

#include "mgos.h"
#include "mgos_gpio.h"
#include "mgos_rpc.h"
#include "frozen.h"
#include "mgos_uart.h"
#include "mgos_mqtt.h"


#define LED_PIN 2
#define UART_NO 1
//#define BUF_SIZE 63
#define NAME_LEN 63

// THE FOLLOWING SHOULD MATCH THE "mqtt.client_id" IN mos.yml
#define ROBOT_NAME "DM_ROBOT"


// DEFINE ONE OF THE FOLLOWING TO CHANGE BEHAVIOUR 
// _LEADER - publishes any remote command it receives to a mqtt topic
// _FOLLOWER - receives commands on the mqtt subscribed topic
// if both undefined, won't do any of the above ie, only way to interact with it
// is via RPC commands: Robot.CmD and Robot.Sensors 
//#define _LEADER
#define _FOLLOWER

// OPTIONAL PARAMTERS
#define _QUEUE
//#define _DISABLE_UART_RX

// THE FOLLOWING HAS BEEN DEPRECATED.
// if _TELEMETRY is defined, it will send telemetry to topic /telemetry
// This should go with _FOLLOWER
//#define _TELEMETRY
//#define TOPIC "/telemetry"

enum {
  ERROR_UNKNOWN_COMMAND = -1,
  ERROR_I2C_NOT_CONFIGURED = -2,
  ERROR_I2C_READ_LIMIT_EXCEEDED = -3
};

// name of current participant
static char name[NAME_LEN + 1];
static int leftObstacle, rightObstacle = 0;
static struct mbuf cmdBuffer;

// subscribe to a topic
static void sub(struct mg_connection *c, const char *fmt, ...) {
  char buf[100];
  struct mg_mqtt_topic_expression te = {.topic = buf, .qos = 1};
  uint16_t sub_id = mgos_mqtt_get_packet_id();
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  mg_mqtt_subscribe(c, &te, 1, sub_id);
  LOG(LL_INFO, ("Subscribing to %s (id %u)", buf, sub_id));
}

// publish a message to a topic
static void pub(const char *fmt, ...) {
  char msg[200];
  memset(msg, 0, sizeof(msg));
  struct json_out jmo = JSON_OUT_BUF(msg, sizeof(msg));
  va_list ap;
  int n;
  va_start(ap, fmt);
  n = json_vprintf(&jmo, fmt, ap);
  va_end(ap);
  mgos_mqtt_pub(mgos_sys_config_get_mqtt_pub(), msg, n, MG_MQTT_QOS(1), false);
  LOG(LL_INFO, ("%s -> %s",mgos_sys_config_get_mqtt_pub(), msg));
}

// execute command action
void process_cmd(const char *cmd) {
  char * tmp_name = NULL;
  size_t charsToSend = 0;
  //int64_t time;

  LOG(LL_INFO, ("Received command: %s", cmd));
  mgos_gpio_toggle(LED_PIN);

#ifdef _QUEUE
  // queue command first
  mbuf_append(&cmdBuffer, cmd, strlen(cmd));

  // send as much as possible
  charsToSend = mgos_uart_write_avail(UART_NO);
  if (cmdBuffer.len < charsToSend) charsToSend = cmdBuffer.len;

  // write to UART 
  if (charsToSend > 0) {
    mgos_uart_write(UART_NO, cmdBuffer.buf, charsToSend);
    mgos_uart_flush(UART_NO);
    mbuf_remove(&cmdBuffer, charsToSend);
  }
#else
    mgos_uart_write(UART_NO, cmd, strlen(cmd));
    mgos_uart_flush(UART_NO);
#endif

  // check for name command to save name of participant
  tmp_name = strrchr(cmd, ' ');
  if (tmp_name != NULL) {
    tmp_name++;
    
    // this assumes that a name always starts with a non-digit
    if (tmp_name[0] < '0' || tmp_name[0] > '9') {
      strncpy(name, tmp_name, NAME_LEN);
      name[strlen(name) - 1] = 0;
      LOG(LL_INFO, ("name is: %s", name));
    }
  }
#ifdef _LEADER
  // send movement mqtt message to "publish" topic  
  LOG(LL_INFO, ("Publishing command: %s", cmd));

  pub("{ id : %Q, cmd : %Q, time : \"%lld\" }", name,  cmd, mgos_uptime_micros());
#endif
}

// process rpc cmd request
static void cmd_cb(struct mg_rpc_request_info *ri, void *cb_arg,
                   struct mg_rpc_frame_info *fi, struct mg_str args) {
  char *cmd = NULL;
  if (json_scanf(args.p, args.len, ri->args_fmt, &cmd) == 1) {
    process_cmd(cmd);
    mg_rpc_send_responsef(ri, NULL);
    free(cmd);
  } else {
    mg_rpc_send_errorf(ri, -1, "Bad request. Expected: {\"cmd\": \"s\"}");
  }
  (void) cb_arg;
  (void) fi;
}

// process rpc sensors request
static void sensors_cb(struct mg_rpc_request_info *ri, void *cb_arg,
                   struct mg_rpc_frame_info *fi, struct mg_str args) {
  mg_rpc_send_responsef(ri, "{ id: %Q, leftObstacle: %d, rightObstacle: %d}", name, leftObstacle, rightObstacle);
  (void) cb_arg;
  (void) fi;
}

// assemble message from the robot serial port
static void uart_dispatcher(int uart_no, void *arg) {
  static struct mbuf lb = {0};
  assert(uart_no == UART_NO);
  size_t rx_av = mgos_uart_read_avail(uart_no);
  if (rx_av == 0) return;
  mgos_uart_read_mbuf(uart_no, &lb, rx_av);
  /* Handle all the wonderful possibilities of different line endings. */
  struct mg_str b = mg_mk_str_n(lb.buf, lb.len);
  char *le = (char *) mg_strchr(b, ';');
  if (le == NULL) return;
  *le = '\0';
  size_t llen = le - lb.buf;
  if (llen == 0) return;
  struct mg_str line = mg_mk_str_n(lb.buf, llen);
  
  /*
   * Complete command recevied from robot base.
   */
  LOG(LL_INFO, ("UART%d> '%.*s'", uart_no, (int) line.len, line.p));
  json_scanf(line.p, line.len, "{ leftObstacle: %d, rightObstacle: %d }", &leftObstacle, &rightObstacle);
  LOG(LL_INFO, ("leftObstacle: %d, rightObstacle: %d", leftObstacle, rightObstacle));
  
  //mgos_mqtt_pub(TOPIC, line.p, line.len, MG_MQTT_QOS(1), false);

  /* Finally, remove the line data from the buffer. */
  mbuf_remove(&lb, llen + 1);

  (void) arg;
}

// MQTT global handler of events
static void mqtt_handler(struct mg_connection *c, int ev, void *p,
                       void *user_data) {
  struct mg_mqtt_message *msg = (struct mg_mqtt_message *) p;
  char *cmd = NULL;

  if (ev == MG_EV_MQTT_CONNACK) {
    LOG(LL_INFO, ("CONNACK: %d", msg->connack_ret_code));
    if (mgos_sys_config_get_mqtt_sub() == NULL ||
        mgos_sys_config_get_mqtt_pub() == NULL) {
      LOG(LL_ERROR, ("Run 'mgos config-set mqtt.sub=... mqtt.pub=...'"));
    } else {
#ifdef _FOLLOWER
      sub(c, "%s", mgos_sys_config_get_mqtt_sub());
#else
      LOG(LL_INFO, ("Not sucsbribing to any MQTT topic"));
#endif
    }
  } else if (ev == MG_EV_MQTT_SUBACK) {
    LOG(LL_INFO, ("Subscription %u acknowledged", msg->message_id));
  } else if (ev == MG_EV_MQTT_PUBLISH) {
    struct mg_str *s = &msg->payload;
    struct json_token t = JSON_INVALID_TOKEN;

    LOG(LL_INFO, ("got command: [%.*s]", (int) s->len, s->p));
    /* Our subscription is at QoS 1, we must acknowledge messages sent to us. */
    mg_mqtt_puback(c, msg->message_id);
    if (json_scanf(s->p, s->len, "{ cmd: %Q }", &cmd) == 1) {
      /* process received command */
#ifdef _FOLLOWER
      // process the command only if this is a _FOLLOWER
      process_cmd(cmd);
#endif
      free(cmd);
    } else {
      pub("{error: {cmd: %Q}}", ERROR_UNKNOWN_COMMAND,
          "unknown command");
    }
  }
  (void) user_data;
}

// startup initialisation
enum mgos_app_init_result mgos_app_init(void) {

  struct mgos_uart_config ucfg;
  mgos_uart_config_set_defaults(UART_NO, &ucfg);
  /*
   * At this point it is possible to adjust baud rate, pins and other settings.
   * 115200 8-N-1 is the default mode, but we set it anyway
   */
  ucfg.baud_rate = 19200;
  ucfg.num_data_bits = 8;
  ucfg.parity = MGOS_UART_PARITY_NONE;
  ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
  if (!mgos_uart_configure(UART_NO, &ucfg)) {
    return MGOS_APP_INIT_ERROR;
  }
  mgos_uart_set_dispatcher(UART_NO, uart_dispatcher, NULL /* arg */);
  #ifdef _DISABLE_UART_RX
  mgos_uart_set_rx_enabled(UART_NO, false);
  #else
  mgos_uart_set_rx_enabled(UART_NO, true);
  #endif

  // LED_PIN toggles on receiving a command ws or otherwise
  mgos_gpio_setup_output(LED_PIN, 0);
  mg_rpc_add_handler(mgos_rpc_get_global(), "Robot.Cmd", "{cmd: %Q}", cmd_cb, NULL);
  mg_rpc_add_handler(mgos_rpc_get_global(), "Robot.Sensors", NULL, sensors_cb, NULL);

  // MQTT init
  mgos_mqtt_add_global_handler(mqtt_handler, NULL);

  strcpy(name, ROBOT_NAME);
  mbuf_init(&cmdBuffer, 512);

  return MGOS_APP_INIT_SUCCESS;
}
