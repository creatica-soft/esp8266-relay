/*
=== Relay controlled by SNTP timer (RTOS SDK based) ===

Install toolchain for crosscompiling (may not be easy, old esp-open-sdk toolchain works fine despite of warnings):
 - supported toolchain version is 1.22.0-92-g8facf4c
 - supported compiler version 5.2.0

 export PATH=~/esp-open-sdk/xtensa-lx106-elf/bin:$PATH

cd ~
wget https://github.com/espressif/ESP8266_RTOS_SDK/releases/download/v3.3/ESP8266_RTOS_SDK-v3.3.zip
unzip ESP8266_RTOS_SDK-v3.3.zip
mkdir -p ESP8266_RTOS_SDK/timer/main

Review and update defines in timer.c

cp timer.c ESP8266_RTOS_SDK/timer/main
cp ESP8266_RTOS_SDK/examples/get-started/project-template/Makefile ESP8266_RTOS_SDK/timer
touch ESP8266_RTOS_SDK/timer/main/component.mk

export IDF_PATH=~/ESP8266_RTOS_SDK

make menuconfig # to produce sdkconfig

make

Power off, ground GPIO0 before flashing, power on

make flash

Power off, unground GPIO0 for normal operation, power on

Run terminal emulator to see the ESP_LOGx(TAG,) messages:
miniterm --raw /dev/ttyAMA1 74880

I (40) boot: ESP-IDF  2nd stage bootloader
I (40) boot: compile time 10:12:18
I (47) qio_mode: Enabling default flash chip QIO
I (47) boot: SPI Speed      : 40MHz
I (48) boot: SPI Mode       : QIO
I (52) boot: SPI Flash Size : 1MB
I (56) boot: Partition Table:
I (60) boot: ## Label            Usage          Type ST Offset   Length
I (67) boot:  0 nvs              WiFi data        01 02 00009000 00004000
I (75) boot:  1 otadata          OTA data         01 00 0000d000 00002000
I (82) boot:  2 phy_init         RF data          01 01 0000f000 00001000
I (90) boot:  3 ota_0            OTA app          00 10 00010000 00070000
I (97) boot:  4 ota_1            OTA app          00 11 00080000 00070000
I (104) boot: End of partition table
I (18024) esp_image: segment 0: paddr=0x00010010 vaddr=0x40210010 size=0x3d2f0 (250608) map
I (18067) esp_image: segment 1: paddr=0x0004d308 vaddr=0x4024d300 size=0x09be4 ( 39908) map
I (18075) esp_image: segment 2: paddr=0x00056ef4 vaddr=0x3ffe8000 size=0x005b0 (  1456) load
I (18075) esp_image: segment 3: paddr=0x000574ac vaddr=0x40100000 size=0x008c0 (  2240) load
I (18084) esp_image: segment 4: paddr=0x00057d74 vaddr=0x401008c0 size=0x057f8 ( 22520) load
I (18097) boot: Loaded app from partition at offset 0x10000
I (27860) system_api: Base MAC address is not set, read default base MAC address from EFUSE
I (27864) system_api: Base MAC address is not set, read default base MAC address from EFUSE
phy_version: 1159.0, 85b471e, Apr 21 2020, 17:03:08, RTOS new
I (27927) phy_init: phy ver: 1159_0
I (27930) reset_reason: RTC reset 2 wakeup 0 store 3, reason is 2
*/

//#define NDEBUG //to get rid of ESP_ERROR_CHECK asserts and subsequent aborts() which will exit the task loop
               // and eventually (in 15sec) causes the task to be killed by watchdog timer because the timer is not reset
               // as the task loop is gone. Perhaps, ESP_ERROR_CHECK is not the correct macro for tasks
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <ctype.h>
#include <netdb.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_system.h"
#include "esp_wifi_types.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "lwip/netif.h"
#include "lwip/netdb.h"
#include "lwip/dhcp.h"
#include "lwip/apps/sntp.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
//#include "rom/gpio.h"
//#include "hw_timer.h"

#define SSID "ssid"
#define PASSPHRASE "password"
//iw wlan0 station dump to check the signal >= -50dBm is good 
// WIFI_POWER values, range is from -128 to 127, level0 - is the maximum power
// from esp_wifi.h:
// [78, 127]: level0
// [76, 77]: level1
// [74, 75] : level2
// [68, 73] : level3
// [60, 67] : level4
// [52, 59] : level5
// [44, 51] : level5 -2dBm
// [34, 43] : level5 -4.5dBm
// [28, 33] : level5 -6dBm
// [20, 27] : level5 -8dBm
// [8, 19] : level5 -11dBm
// [-128, 7] : level5 -14dBm
// https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-reference/wifi/esp_wifi.html gives different numbers
// [82, 127]: level0
// [78, 81] : level1
// [74, 77] : level2
// // [68, 73] : level3
// [64, 67] : level4
// [56, 63] : level5
// [49, 55] : level5 - 2dBm
// [33, 48] : level5 - 6dBm
// [25, 32] : level5 - 8dBm
// [13, 24] : level5 - 11dBm
// [1, 12] : level5 - 14dBm
// [-128, 0] : level5 - 17.5dBm
#define WIFI_POWER 7
//wifi modes: //WIFI_PROTOCOL_11B, WIFI_PROTOCOL_11G, WIFI_PROTOCOL_11N, can be ORed (|)
//Currently only 802.11b or 802.11bg or 802.11bgn modes are supported
#define WIFI_MODE WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G
#define SSID_CHANNEL 5 //optional channel for AP scan
#define HOSTNAME "simple-switch"
//Station static IP config
#define USE_STATIC_IP true
#define STATIC_IP "192.168.1.100"
#define NETMASK "255.255.255.0"
#define GATEWAY_IP "192.168.1.1"
#define NTP0 "0.pool.ntp.org"
#define NTP1 "1.pool.ntp.org"
#define NTP2 "2.pool.ntp.org"
#define DNS1 "192.168.1.1"
#define DNS2 "8.8.8.8"
#define TIMEZONE "NZST-12NZDT-13,M10.1.0,M3.3.0" //for TZ format see man tzset()
#define LOCAL_UDP_PORT 7777 //send 0 or 1 to this port to turn relay off or on using for example, netcat:
                            //echo -n 0|nc -un -w 1 192.168.1.100 7777

#define RELAY_TASK_INTERVAL 1000 //ms
#define RELAY_TASK_PRIORITY 5
//whether the relay is on (0 - GPIO_NUM_0 low level) or off (1 - GPIO_NUM_0 high level) by default
#define RELAY_DEFAULT_STATE 1

#define ESP_CONFIG_TASK_DELAY_MS 1000
#define ESP_CONFIG_TASK_PRIORITY 5
#define AP_CHECK_TASK_PRIORITY 3
#define AP_CHECK_TASK_DELAY_MS 60000

//task stack sizes - adjust if you see high water mark warnings or errors in the log
#define ESP_CONFIG_TASK_SS 2000
#define RELAY_TASK_SS 1200
#define AP_CHECK_TASK_SS 1000
#define HIGH_WATER_MARK_WARNING 50
#define HIGH_WATER_MARK_CRITICAL 10

static const char *TAG = HOSTNAME;
static TaskHandle_t esp_config_task_handle = NULL;

static char * authmode(wifi_auth_mode_t mode) {
	switch (mode) {
	case WIFI_AUTH_OPEN:
		return "open";
	case WIFI_AUTH_WEP:
		return "wep";
	case WIFI_AUTH_WPA_PSK:
		return "wpa_psk";
	case WIFI_AUTH_WPA2_PSK:
		return "wpa2_psk";
	case WIFI_AUTH_WPA_WPA2_PSK:
		return "wpa_wpa2_psk";
	case WIFI_AUTH_WPA2_ENTERPRISE:
		return "wpa2_enterprise";
	case WIFI_AUTH_MAX:
		return "max";
	default:
		return "unknown";
	}
}
static void sysinfo(void) {
	ESP_LOGI(TAG, "system IDF version %s", esp_get_idf_version());
	ESP_LOGI(TAG, "free heap size %u", esp_get_free_heap_size());
}

static void relay_task(void *arg) {
	char datetime[64];
	char buffer[32];
	int err, sock;
	time_t now;
	struct tm timeinfo;
	UBaseType_t hwm;
	struct sockaddr_in local_addr = {
		.sin_family = AF_INET,
		.sin_addr.s_addr = htonl(INADDR_ANY),
		.sin_port = htons(LOCAL_UDP_PORT)
	};

	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock >= 0) {
		err = bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
		if (err < 0) {
			printf("relay_task: bind failed - %s, remote switch will not work\n", strerror(errno));
			close(sock);
			vTaskDelete(NULL);
			return;
		}
	}
	else printf("relay_task: socket failed - %s, remote switch will not work\n", strerror(errno));

	while (1) {
		now = 0;
		memset(&timeinfo, 0, sizeof(timeinfo));
		memset(buffer, 0, sizeof(buffer));
		time(&now);
		localtime_r(&now, &timeinfo);
		strftime(datetime, sizeof(datetime), "%a %b %d %H:%M:%S %Y", &timeinfo);
		ESP_LOGI(TAG, "relay_task: datetime %s", datetime);

		err = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, NULL, NULL);
		if (err < 0) {
			ESP_LOGE(TAG, "relay_task: recvfrom failed - %s", strerror(errno));
			goto sleep;
		}
		else if (err > 1) {
			ESP_LOGE(TAG, "relay_task: recvfrom data is too long %d, expect 1", err);
			goto sleep;
		}
		else if (err == 1) {
			buffer[err] = 0;
			if (isdigit((unsigned int)buffer[0])) {
				if (atoi(buffer)) {
					ESP_LOGI(TAG, "relay_task: turning relay on...");
					gpio_set_level(GPIO_NUM_0, 0); //Set GPIO0 as low - level output.
				}
				else {
					ESP_LOGI(TAG, "relay_task: turning relay off...");
					gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
				}
			}
			else ESP_LOGE(TAG, "relay_task: recvfrom data is not a digit %s", buffer);
		}

	sleep:
		hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "relay_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "relay_task: uxTaskGetStackHighWaterMark %lu", hwm);

		vTaskDelay(pdMS_TO_TICKS(RELAY_TASK_INTERVAL));
	}
}

static void esp_config_task(void *arg)
{
	esp_err_t err;
	uint32_t notificationValue;

	while (1) {
		notificationValue = ulTaskNotifyTake(1, pdMS_TO_TICKS(ESP_CONFIG_TASK_DELAY_MS));
		if (notificationValue) {
			sysinfo();
			ESP_LOGI(TAG, "esp_config_task: setting hostname %s...", HOSTNAME);
			err = tcpip_adapter_set_hostname(WIFI_IF_STA, HOSTNAME);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_hostname failed - %s", esp_err_to_name(err));

			if (USE_STATIC_IP) {
				tcpip_adapter_dhcp_status_t status;
				tcpip_adapter_dhcps_get_status(TCPIP_ADAPTER_IF_STA, &status);
				//ESP_LOGI(TAG, "esp_config_task: tcpip_adapter_dhcps_get_status %u", status);
				if (status != TCPIP_ADAPTER_DHCP_STOPPED) { //tcpip_adapter_dhcps_get_status does not return TCPIP_ADAPTER_DHCP_STARTED but TCPIP_ADAPTER_DHCP_INIT
					ESP_LOGI(TAG, "esp_config_task: USE_STATIC_IP is true, stopping dhcp client...");
					err = tcpip_adapter_dhcpc_stop(WIFI_IF_STA);
					if (err != ESP_OK)
						ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_dhcpc_stop failed - %s", esp_err_to_name(err));
				}
				tcpip_adapter_ip_info_t ip;
				ipaddr_aton(STATIC_IP, &ip.ip);
				ipaddr_aton(GATEWAY_IP, &ip.gw);
				ipaddr_aton(NETMASK, &ip.netmask);

				ESP_LOGI(TAG, "esp_config_task: setting static ip %s...", ipaddr_ntoa(&ip.ip));
				err = tcpip_adapter_set_ip_info(WIFI_IF_STA, &ip);
				if (err != ESP_OK)
					ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_ip_info failed - %s", esp_err_to_name(err));
			}

			tcpip_adapter_dns_info_t prim_dns_ip;
			tcpip_adapter_dns_info_t sec_dns_ip;

			ipaddr_aton(DNS1, &prim_dns_ip.ip);
			ipaddr_aton(DNS2, &sec_dns_ip.ip);
			ESP_LOGI(TAG, "esp_config_task: setting primary dns %s...", DNS1);
			err = tcpip_adapter_set_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &prim_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_dns_info failed - %s", esp_err_to_name(err));
			ESP_LOGI(TAG, "esp_config_task: setting secondary dns %s...", DNS2);
			err = tcpip_adapter_set_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_BACKUP, &sec_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_dns_info failed - %s", esp_err_to_name(err));
			err = tcpip_adapter_get_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &prim_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_get_dns_info failed - %s", esp_err_to_name(err));
			err = tcpip_adapter_get_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_BACKUP, &sec_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_get_dns_info failed - %s", esp_err_to_name(err));

			ESP_LOGI(TAG, "esp_config_task: initializing SNTP...");
			sntp_setoperatingmode(SNTP_OPMODE_POLL);
			sntp_setservername(0, NTP0);
			sntp_setservername(1, NTP1);
			sntp_setservername(2, NTP2);
			sntp_init();

			ESP_LOGI(TAG, "sntp 0: %s", sntp_getservername(0));
			ESP_LOGI(TAG, "sntp 1: %s", sntp_getservername(1));
			ESP_LOGI(TAG, "sntp 2: %s", sntp_getservername(2));

			ESP_LOGI(TAG, "esp_config_task: setting timezone %s...", TIMEZONE);
			setenv("TZ", TIMEZONE, 1);
			tzset();

			ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT));
			ESP_LOGI(TAG, "esp_config_task: setting relay default state %d...", RELAY_DEFAULT_STATE);
			gpio_set_level(GPIO_NUM_0, RELAY_DEFAULT_STATE);

			if (pdPASS != xTaskCreate(&relay_task, "relay_task", RELAY_TASK_SS, NULL, RELAY_TASK_PRIORITY, NULL))
				ESP_LOGE(TAG, "esp_config_task: failed to create relay task");
		}

		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "esp_config_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "esp_config_task: uxTaskGetStackHighWaterMark %lu", hwm);
	}
}

static esp_err_t event_handler(void *ctx, system_event_t *evt)
{
	//ESP_LOGI(TAG,"event %x", evt->event_id);
	char * reason = "";
	esp_err_t err;

	switch (evt->event_id) {
	case SYSTEM_EVENT_WIFI_READY: //should be triggered after esp-wifi_init() but it is not!
		//ESP_LOGI(TAG, "event_handler: wifi is ready");
		break;
	case SYSTEM_EVENT_SCAN_DONE:
		//ESP_LOGI(TAG, "event_handler: scan is done");
		break;
	case SYSTEM_EVENT_STA_START:
		//ESP_LOGI(TAG, "wifi station started");
		err = esp_wifi_set_max_tx_power(WIFI_POWER); //should be called after esp_wifi_start()
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_set_max_tx_power failed - %s", esp_err_to_name(err));

		uint8_t wifi_mode = WIFI_MODE;
		err = esp_wifi_set_protocol(ESP_IF_WIFI_STA, wifi_mode);
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_set_protocol failed - %s", esp_err_to_name(err));
		switch (wifi_mode) {
		case WIFI_PROTOCOL_11B:
			ESP_LOGI(TAG, "wifi mode %u means 802.11b (<11Mbps, 35-140m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11G:
			ESP_LOGI(TAG, "wifi mode %u means 802.11g (<54Mbps, 38-140m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11N:
			ESP_LOGI(TAG, "wifi mode %u means 802.11n (<289/600Mbps, 70-250m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G:
			ESP_LOGI(TAG, "wifi mode %u means 802.11bg (<54Mbps, 35-140m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N:
			ESP_LOGI(TAG, "wifi mode %u means 802.11bgn (<289/600Mbps, 35-250m)", WIFI_MODE);
			break;
		default:
			ESP_LOGI(TAG, "wifi mode is unknown %u", wifi_mode);
		}

		wifi_config_t wifi_config = {
			.sta = {
				.ssid = SSID,
				.password = PASSPHRASE,
				.bssid_set = 0,
#ifdef SSID_CHANNEL
				.scan_method = WIFI_FAST_SCAN,
				.channel = SSID_CHANNEL
#else
				.scan_method = WIFI_ALL_CHANNEL_SCAN,
				.channel = 0
#endif // SSID_CHANNEL
			},
		};
		//ESP_LOGI(TAG, "Configuring esp for scan method %u, channel %u...", wifi_config.sta.scan_method, wifi_config.sta.channel);
		err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_set_config failed - %s", esp_err_to_name(err));
		//ESP_LOGI(TAG, "connecting to ap %s...", SSID);
		err = esp_wifi_connect();
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_connect failed - %s", esp_err_to_name(err));
		break;
	case SYSTEM_EVENT_STA_STOP:
		ESP_LOGW(TAG, "event_handler: wifi stopped, restarting...");
		esp_restart();
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
		ESP_LOGI(TAG, "event_handler: connected to ssid %s, channel %d, authmode %s",
			evt->event_info.connected.ssid,
			evt->event_info.connected.channel, authmode(evt->event_info.connected.authmode));
		if (USE_STATIC_IP) {
			if (esp_config_task_handle) {
				//ESP_LOGI(TAG, "event_handler: notifying esp_config_task...");
				xTaskNotifyGive(esp_config_task_handle);
			}
			else ESP_LOGE(TAG, "event_handler: esp_config_task_handle is NULL");
		}
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		switch (evt->event_info.disconnected.reason) {
		case WIFI_REASON_UNSPECIFIED: reason = "unknown"; break;
		case WIFI_REASON_AUTH_EXPIRE: reason = "auth expired"; break;
		case WIFI_REASON_AUTH_LEAVE: reason = "left authentication"; break;
		case WIFI_REASON_ASSOC_EXPIRE: reason = "assoc expired"; break;
		case WIFI_REASON_ASSOC_TOOMANY: reason = "too many assoc"; break;
		case WIFI_REASON_NOT_AUTHED: reason = "not authenticated"; break;
		case WIFI_REASON_NOT_ASSOCED: reason = "not associated"; break;
		case WIFI_REASON_ASSOC_LEAVE: reason = "left association"; break;
		case WIFI_REASON_ASSOC_NOT_AUTHED: reason = "association not authenticated"; break;
		case WIFI_REASON_DISASSOC_PWRCAP_BAD: reason = "disassociated - bad pwr cap"; break;
		case WIFI_REASON_DISASSOC_SUPCHAN_BAD: reason = "disassociated - bad sup channel"; break;
		case WIFI_REASON_IE_INVALID: reason = "invalid IE"; break;
		case WIFI_REASON_MIC_FAILURE: reason = "misc failure"; break;
		case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: reason = "4-way handshake timeout"; break;
		case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT: reason = "group key update timeout"; break;
		case WIFI_REASON_IE_IN_4WAY_DIFFERS: reason = "IE in 4-way differs"; break;
		case WIFI_REASON_GROUP_CIPHER_INVALID: reason = "invalid group cipher"; break;
		case WIFI_REASON_PAIRWISE_CIPHER_INVALID: reason = "invalid pairwise cipher"; break;
		case WIFI_REASON_AKMP_INVALID: reason = "invalid AKMP"; break;
		case WIFI_REASON_UNSUPP_RSN_IE_VERSION: reason = "unsupported RSN IE version"; break;
		case WIFI_REASON_INVALID_RSN_IE_CAP: reason = "invalid RSN IE cap"; break;
		case WIFI_REASON_802_1X_AUTH_FAILED: reason = "802.1x auth failed"; break;
		case WIFI_REASON_CIPHER_SUITE_REJECTED: reason = "cipher suite rejected"; break;
		case WIFI_REASON_BEACON_TIMEOUT: reason = "beacon timeout"; break;
		case WIFI_REASON_NO_AP_FOUND: reason = "no AP found"; break;
		case WIFI_REASON_AUTH_FAIL: reason = "authentication failed"; break;
		case WIFI_REASON_ASSOC_FAIL: reason = "association failed"; break;
		case WIFI_REASON_HANDSHAKE_TIMEOUT: reason = "handshake timeout"; break;
		}
		ESP_LOGW(TAG, "event_handler: disconnected from ssid %s, reason %s, re-connecting...",
			evt->event_info.disconnected.ssid, reason);
		ESP_LOGW(TAG, "event_handler: out of precausion turning relay off...");
		gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
		//ESP_LOGI(TAG, "connecting to ap %s...", SSID);
		err = esp_wifi_connect();
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_connect failed - %s", esp_err_to_name(err));
		break;
	case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
		ESP_LOGW(TAG, "event_handler: station authmode changed: %d -> %d, restarting...",
			evt->event_info.auth_change.old_mode,
			evt->event_info.auth_change.new_mode);
		esp_restart();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		if (!USE_STATIC_IP) {
			ESP_LOGI(TAG, "event_handler: station got ip:" IPSTR ", mask:" IPSTR ", gw:" IPSTR,
				IP2STR(&evt->event_info.got_ip.ip_info.ip),
				IP2STR(&evt->event_info.got_ip.ip_info.netmask),
				IP2STR(&evt->event_info.got_ip.ip_info.gw));
			if (esp_config_task_handle) {
				//ESP_LOGI(TAG, "event_handler: notifying esp_config_task...");
				xTaskNotifyGive(esp_config_task_handle);
			}
			else ESP_LOGE(TAG, "event_handler: esp_config_task_handle is NULL");
		}		
		break;
	case SYSTEM_EVENT_STA_LOST_IP:
		ESP_LOGE(TAG, "event_handler: station lost ip, restarting...");
		esp_restart();
		break;
	default:
		ESP_LOGI(TAG, "event_handler: unknown event with id %x", evt->event_id);
		break;
	}
	return ESP_OK;
}

static void ap_check_task(void *arg) {
	wifi_ap_record_t ap_record;
	esp_err_t err;
	UBaseType_t hwm;
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(AP_CHECK_TASK_DELAY_MS));

		err = esp_wifi_sta_get_ap_info(&ap_record);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "ap_check_task: esp_wifi_sta_get_ap_info failed - %s, restarting...", esp_err_to_name(err));
			esp_restart();
		}

		hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "ap_check_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "ap_check_task: uxTaskGetStackHighWaterMark %lu", hwm);
	}
}

void app_main(void) {
	esp_err_t err;

	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
		// OTA app partition table has a smaller NVS partition size than the non-OTA
		// partition table. This size mismatch may cause NVS initialization to fail.
		// If this happens, we erase NVS partition and initialize NVS again.
		err = nvs_flash_erase();
		if (err != ESP_OK)
			ESP_LOGE(TAG, "app_main: nvs_flash_erase failed - %s", esp_err_to_name(err));
		else {
			err = nvs_flash_init();
			if (err != ESP_OK)
				ESP_LOGE(TAG, "app_main: nvs_flash_init2 failed - %s", esp_err_to_name(err));
		}
	} 
	else if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: nvs_flash_init1 failed - %s", esp_err_to_name(err));

	tcpip_adapter_init(); //enables tcpip stack, see tcpip_adapter.h
	
	err = esp_event_loop_init(event_handler, NULL); //see esp_event_loop.h
	if (err != ESP_OK) 
		ESP_LOGE(TAG, "app_main: esp_event_loop_init failed - %s", esp_err_to_name(err));

	//ESP_LOGI(TAG, "Calling esp_wifi_init()...");
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	err = esp_wifi_init(&cfg); //init event queue according to esp_wifi.h
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_init failed - %s", esp_err_to_name(err));

	//ESP_LOGI(TAG, "setting esp to station mode...");
	err = esp_wifi_set_mode(WIFI_MODE_STA);
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_set_mode failed - %s", esp_err_to_name(err));

	if (pdPASS != xTaskCreate(&esp_config_task, "esp_config_task", ESP_CONFIG_TASK_SS, NULL, ESP_CONFIG_TASK_PRIORITY, &esp_config_task_handle))
		ESP_LOGE(TAG, "app_main: failed to create esp config task");

	if (pdPASS != xTaskCreate(&ap_check_task, "ap_check_task", AP_CHECK_TASK_SS, NULL, AP_CHECK_TASK_PRIORITY, NULL))
		ESP_LOGE(TAG, "app_main: failed to create AP check task");

	err = esp_wifi_start();
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_start failed - %s", esp_err_to_name(err));
}
