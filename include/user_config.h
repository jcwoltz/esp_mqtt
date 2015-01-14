#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#define CFG_HOLDER	0x00FF55A4
#define CFG_LOCATION	0x3C	/* Please don't change or if you know what you doing */

/*DEFAULT CONFIGURATIONS*/

#define MQTT_HOST			"util1.wocspool.com" //or "192.168.11.1"
#define MQTT_PORT			1883
#define MQTT_BUF_SIZE		1024
#define MQTT_KEEPALIVE		120	 /*second*/

#define MQTT_CLIENT_ID		"jch_%08X"
#define MQTT_USER			"JCH_node1"
#define MQTT_PASS			"abc123"


#define STA_SSID "ocs9"
#define STA_PASS "BasketBall4All"
#define STA_TYPE AUTH_WPA2_PSK

#define MQTT_RECONNECT_TIMEOUT 	5	/*second*/

#define CLIENT_SSL_ENABLE


#endif
