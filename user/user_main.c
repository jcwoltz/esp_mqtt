/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
//#include "driver/dht22.h"
#include "driver/gpio16.h"

MQTT_Client mqttClient;
#define DELAY 4000 /* milliseconds */
#define user_procTaskPrio        3
#define user_procTaskQueueLen    1

LOCAL os_event_t    user_procTaskQueue[user_procTaskQueueLen];
LOCAL os_timer_t dht22_timer;
//scl gpio14 pin17
//sda gpio2 pin20
//dht gpio12 pin 16

#define LED1_GPIO 4	//pin 22 nothing else attached
#define LED1_GPIO_MUX PERIPHS_IO_MUX_GPIO4_U
#define LED1_GPIO_FUNC FUNC_GPIO4

#define LED2_GPIO 5	//pin 11 nothing else attached
#define LED2_GPIO_MUX PERIPHS_IO_MUX_GPIO5_U
#define LED2_GPIO_FUNC FUNC_GPIO5

#define LED3_GPIO 15	//pin 19 2k gnd
#define LED3_GPIO_MUX PERIPHS_IO_MUX_MTDO_U
#define LED3_GPIO_FUNC FUNC_GPIO15

#define LED4_GPIO 13	//pin 18 nothing else attached
#define LED4_GPIO_MUX PERIPHS_IO_MUX_MTCK_U
#define LED4_GPIO_FUNC FUNC_GPIO13

#define LED5_GPIO 0	//pin 21 2k pullup
#define LED5_GPIO_MUX PERIPHS_IO_MUX_GPIO0_U
#define LED5_GPIO_FUNC FUNC_GPIO0

#define LED_GPIO LED5_GPIO
#define LED_GPIO_MUX LED5_GPIO_MUX
#define LED_GPIO_FUNC LED5_GPIO_FUNC



static void user_procTask(os_event_t *events);
static volatile os_timer_t some_timer;

#ifdef __DHT22_H__
LOCAL void ICACHE_FLASH_ATTR dht22_cb(void *arg)
{
 struct dht_sensor_data* r = DHTRead();
 float lastTemp = r->temperature;
 float lastHum = r->humidity;
 char str[64];

 if(r->success)
 {
 INFO("Temperature: %d.%d *C, Humidity: %d.%d %%\r\n", (int)(lastTemp),(int)((lastTemp - (int)lastTemp)*100), (int)(lastHum),(int)((lastHum - (int)lastHum)*100));
 os_sprintf(str,"%d.%d", (int)(lastTemp),(int)((lastTemp - (int)lastTemp)*100));
 MQTT_Publish(&mqttClient, "sensor/node1/dht/t", str, os_strlen(str), 0, 0);
 }
 else
 {
 INFO("Error reading temperature and humidity\r\n");
 }
}
#endif

void some_timerfunc(void *arg)
{
    //Do blinky stuff
    if (GPIO_REG_READ(GPIO_OUT_ADDRESS) & BIT13)
    {
        //Set GPIO2 to LOW
	//GPIO_OUTPUT_SET(LED4_GPIO, 0);
        gpio_output_set(0, BIT13, BIT13, 0);
    }
    else
    {
        //Set GPIO2 to HIGH
	//GPIO_OUTPUT_SET(LED4_GPIO, 1);
        gpio_output_set(BIT13, 0, BIT13, 0);
    }
}

//Do nothing function
static void ICACHE_FLASH_ATTR
user_procTask(os_event_t *events)
{
    os_delay_us(10);
}




void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	INFO("MQTT: Starting subscription\r\n");
	MQTT_Subscribe(client, "sensor/node1/control/led1", 0);
	MQTT_Subscribe(client, "sensor/node1/control/led2", 0);
	MQTT_Publish(client, "sensor/node1/boot", "hello2", 6, 0, 0);
}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char topicBuf[64], dataBuf[64];
	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("MQTT topic: %s, data: %s \r\n", topicBuf, dataBuf);

	/* Echo back to /echo channel*/
	//MQTT_Publish(client, "/echo", dataBuf, data_len, 0, 0);

	if(0 == os_strcmp(topicBuf,"sensor/node1/control/led1")) {
	 INFO("LED1 sub\r\n");
	 if (0 == os_strcmp(dataBuf,"on"))
	 {
	  INFO("LED1 sub data high\r\n");
	  //Set GPIO2 to HIGH
	  //gpio_output_set(BIT2, 0, BIT2, 0);
	  GPIO_OUTPUT_SET(LED1_GPIO, 1);
	 } else if (0 == os_strcmp(dataBuf,"off"))
	 {
	  INFO("LED1 sub data low\r\n");
	  //Set GPIO2 to LOW
	  //gpio_output_set(0, BIT2, BIT2, 0);
	  GPIO_OUTPUT_SET(LED1_GPIO, 0);
	 } else {
	  INFO("\r\n Unable to parse data and set gpio2: \r\n");
	 }
	}

}


void user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	gpio_init();
	PIN_FUNC_SELECT(LED1_GPIO_MUX, LED1_GPIO_FUNC);
	PIN_FUNC_SELECT(LED2_GPIO_MUX, LED2_GPIO_FUNC);
	PIN_FUNC_SELECT(LED3_GPIO_MUX, LED3_GPIO_FUNC);
	PIN_FUNC_SELECT(LED4_GPIO_MUX, LED4_GPIO_FUNC);
	PIN_FUNC_SELECT(LED5_GPIO_MUX, LED5_GPIO_FUNC);
	GPIO_OUTPUT_SET(LED1_GPIO, 0);
	GPIO_OUTPUT_SET(LED2_GPIO, 0);
	GPIO_OUTPUT_SET(LED4_GPIO, 0);
	//DHTInit(DHT11, 2000);

        //Set GPIO2 to output mode
        //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
        //Set GPIO2 low
        //gpio_output_set(0, BIT2, BIT2, 0);

        //Disarm timer
        os_timer_disarm(&some_timer);
        //Setup timer
        os_timer_setfn(&some_timer, (os_timer_func_t *)some_timerfunc, NULL);
        //Arm the timer
        //&some_timer is the pointer
        //1000 is the fire time in ms
        //0 for once and 1 for repeating
	os_timer_arm(&some_timer, 50, 1);
        //Start os task
        system_os_task(user_procTask, user_procTaskPrio,user_procTaskQueue, user_procTaskQueueLen);


	//os_delay_us(4000000);

	CFG_Load();

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, SEC_NONSSL);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
#ifdef __DHT22_H__
os_timer_disarm(&dht22_timer);
os_timer_setfn(&dht22_timer, (os_timer_func_t *)dht22_cb, (void *)0);
os_timer_arm(&dht22_timer, DELAY, 1);
#endif

}
