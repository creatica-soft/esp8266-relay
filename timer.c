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

#define NDEBUG //to get rid of ESP_ERROR_CHECK asserts and subsequent aborts() which will exit the task loop
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
#define MIN_SCANTIME 1000 //wifi min active scan time in ms
#define MAX_SCANTIME 5000 //wifi max active scan time in ms, passive scan should be less than 1500ms to avoid disconnecting from the AP
#define HOSTNAME "cabin-fan"
#define NTP0 "0.pool.ntp.org"
#define NTP1 "1.pool.ntp.org"
#define NTP2 "2.pool.ntp.org"
#define DNS1 "192.168.1.1"
#define DNS2 "8.8.8.8"

//update timer parameters from http://WEB_SERVER:WEB_SERVER_PORT/WEB_SERVER_PATH
#define HTTP_UPDATE true

//check for new timer parameters every ms
#define HTTP_UPDATE_INTERVAL 300000
#define HTTP_UPDATE_TASK_PRIORITY 5 //should be lower than 8
#define WEB_SERVER "example.com"
#define WEB_SERVER_PORT "80"

#define TIMER_ON_CRONLINE "* 0 9,11,13,15,17 * * *" //sec min hour day month day-of-week
#define TIMER_OFF_CRONLINE "* 0 10,12,14,16,18 * * *" //sec min hour day month day-of-week
//timezone from -11 to 13
#define RELAY_TASK_INTERVAL 55000 //period in ms to check if relay should be on or off
								//set it based on relay schedule
#define RELAY_TASK_PRIORITY 6 //should be lower than 8
#define TIMEZONE "NZST-12NZDT-13,M10.1.0,M3.3.0" //for TZ format see man tzset()

/*WEB_SERVER_PATH file should have four lines:
CRONLINE_ON
CRONLINE_OFF
RELAY_TASK_INTERVAL
TIMEZONE
*/
#define WEB_SERVER_PATH "/timer-parameters.txt"
#define HTTP_METHOD "GET"
#define HTTP_VERSION "HTTP/1.0" //HTTP/1.1 may need to deal with Transfer-Encoding
#define USER_AGENT "esp8266-timer"
#define HTTP_TIMEOUT 3 //sec

//OTA update of esp8266 flash with the new image
#define HTTP_FW_UPDATE true
#define FW_CHECK_INTERVAL 900000 //ms
#define FW_UPDATE_TASK_PRIORITY 4
//increase timer version every time you modify this file to avoid re-downloading the same image over and over;
//for example, make it /timer-2.bin but upload the binary to the web server with the previously flashed version, i.e. /timer-1.bin
#define FW_PATH "/timer-1.bin"

//Station static IP config
#define USE_STATIC_IP true
#define STATIC_IP "192.168.1.101"
#define NETMASK "255.255.255.0"
#define GATEWAY_IP "192.168.1.1"

#define MAX_AP_RECORDS 10

#define REMOTE_LOGGING true
#define REMOTE_LOGGING_IP "192.168.1.1"
#define REMOTE_LOGGING_UDP_PORT 6666
#define CHAR_TIMEOUT 1000 //ms, waiting for new log char
#define MAX_LOG_BUFFER_SIZE 1460
#define LOGGING_TASK_PRIORITY 3

#define ESP_CONFIG_TASK_WAIT_NOTIFY_MS 1000

//task stack sizes - adjust if you see warnings or errors in the log
#define REMOTE_LOGGING_TASK_SS 1000
#define ESP_CONFIG_TASK_SS 2000
#define HTTP_UPDATE_TASK_SS 4000
#define FW_UPDATE_TASK_SS 4000
#define RELAY_TASK_SS 1200
#define HIGH_WATER_MARK_WARNING 50
#define HIGH_WATER_MARK_CRITICAL 10

unsigned int relay_task_interval = RELAY_TASK_INTERVAL;
char * tz = TIMEZONE;
int secondsOn[60], minutesOn[60], hoursOn[24], daysOn[31], monthsOn[12], dweekOn[7];
int secondsOff[60], minutesOff[60], hoursOff[24], daysOff[31], monthsOff[12], dweekOff[7];

static const char *TAG = "timer";
static uint8_t log_buffer[MAX_LOG_BUFFER_SIZE];
static uint16_t log_buffer_pointer;
static int sock;
static struct sockaddr_in remote_addr;
static TaskHandle_t esp_config_task_handle = NULL, logging_task_handle = NULL;

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
	//ESP_LOGI(TAG, "cpu frequency %u", rtc_clk_cpu_freq_get());
	//Overclock CPU (default is RTC_CPU_FREQ_80M)
	//rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
	esp_chip_info_t chip;
	esp_chip_info(&chip);
	char * model = "", *flash = "";
	if (chip.model == CHIP_ESP8266) model = "ESP8266";
	else model = "unknown";
	ESP_LOGI(TAG, "chip model %s", model);
	ESP_LOGI(TAG, "chip features %u", chip.features);
	ESP_LOGI(TAG, "chip cores %u", chip.cores);
	ESP_LOGI(TAG, "chip revision %u", chip.revision);

	flash_size_map flash_size = system_get_flash_size_map();
	switch (flash_size) {
	case FLASH_SIZE_4M_MAP_256_256: flash = "4Mbit_256x256"; break;
	case FLASH_SIZE_2M: flash = "2Mbit"; break;
	case FLASH_SIZE_8M_MAP_512_512: flash = "8Mbit_512x512"; break;
	case FLASH_SIZE_16M_MAP_512_512: flash = "16Mbit_512x512"; break;
	case FLASH_SIZE_32M_MAP_512_512: flash = "32Mbit_512x512"; break;
	case FLASH_SIZE_16M_MAP_1024_1024: flash = "16Mbit_1024x1024"; break;
	case FLASH_SIZE_32M_MAP_1024_1024: flash = "32Mbit_1024x1024"; break;
	case FLASH_SIZE_32M_MAP_2048_2048: flash = "32Mbit_2048x2048"; break;
	case FLASH_SIZE_64M_MAP_1024_1024: flash = "64Mbit_1024x1024"; break;
	case FLASH_SIZE_128M_MAP_1024_1024: flash = "128Mbit_1024x1024"; break;
	case FALSH_SIZE_MAP_MAX: flash = "MAX"; break;
	}
	ESP_LOGI(TAG, "system flash size %s", flash);
	ESP_LOGI(TAG, "system IDF version %s", esp_get_idf_version());
	ESP_LOGI(TAG, "free heap size %u", esp_get_free_heap_size());
}
static int cron_parser(char * cronline, bool onTimer) {
	//cronline <seconds> <minutes> <hours> <days> <months> <days-of-the-week>
	char * cron_field[6];
	char * cron_subfield[6][60];
	char * subfield_min = NULL;
	char * subfield_max = NULL;
	int i, j, k, min = 0, max = 0;

	i = 0;
	cron_field[i] = strtok(cronline, " ");
	while (cron_field[i] && i < 5) {
		i++;
		cron_field[i] = strtok(NULL, " ");
	}
	for (i = 0; i < 6; i++) {
		j = 0;
		if (!cron_field[i]) break;
		//ESP_LOGI(TAG,"cron_field[%u] %s", i, cron_field[i]);
		cron_subfield[i][j] = strtok(cron_field[i], ",");
		while (cron_subfield[i][j] && j < 59) {
			j++;
			cron_subfield[i][j] = strtok(NULL, ",");
		}
	}
	for (i = 0; i < 6; i++) {
		for (j = 0; j < 60; j++) {
			if (!cron_subfield[i][j]) break;
			//ESP_LOGI(TAG,"cron_subfield[%u][%u] %s", i, j, cron_subfield[i][j]);
			min = 255;
			subfield_min = strtok(cron_subfield[i][j], "-");
			if (!subfield_min) break;
			//ESP_LOGI(TAG,"subfield_min %s", subfield_min);
			if (strncmp(subfield_min, "*", 1) != 0) {
				if (!isdigit((unsigned char)subfield_min[0])) return 1;
				min = atoi(subfield_min);
				subfield_max = strtok(NULL, "-");
				if (subfield_max) {
					//ESP_LOGI(TAG,"subfield_max %s", subfield_max);
					if (!isdigit((unsigned char)subfield_max[0])) return 2;
					max = atoi(subfield_max);
				}
				else max = min;
			}
			switch (i) {
			case 0: //seconds
				if (min == 255) { min = 0; max = 59; }
				if (min >= 60 || max >= 60) return 3;
				//if (onTimer) printf("cron_parser: secondsOn ");
				//else printf("cron_parser: secondsOff ");
				for (k = min; k <= max; k++) {
					if (onTimer) secondsOn[k] = 1;
					else secondsOff[k] = 1;
					//printf("%d ", k);
				}
				//printf("\n");
				break;
			case 1: //minutes
				if (min == 255) { min = 0; max = 59; }
				if (min >= 60 || max >= 60) return 4;
				//if (onTimer) printf("cron_parser: minutesOn ");
				//else printf("cron_parser: minutesOff ");
				for (k = min; k <= max; k++) {
					if (onTimer) minutesOn[k] = 1;
					else minutesOff[k] = 1;
					//printf("%d ", k);
				}
				//printf("\n");
				break;
			case 2: //hours
				if (min == 255) { min = 0; max = 23; }
				if (min >= 24 || max >= 24) return 5;
				//if (onTimer) printf("cron_parser: hoursOn ");
				//else printf("cron_parser: hoursOff ");
				for (k = min; k <= max; k++) {
					if (onTimer) hoursOn[k] = 1;
					else hoursOff[k] = 1;
					//printf("%d ", k);
				}
				//printf("\n");
				break;
			case 3: //days
				if (min == 255) { min = 1; max = 31; }
				min--; max--;
				if (min >= 31 || max >= 31) return 6;
				//if (onTimer) printf("cron_parser: daysOn ");
				//else printf("cron_parser: daysOff ");
				for (k = min; k <= max; k++) {
					if (onTimer) daysOn[k] = 1;
					else daysOff[k] = 1;
					//printf("%d ", k);
				}
				//printf("\n");
				break;
			case 4: //months
				if (min == 255) { min = 1; max = 12; }
				min--; max--;
				if (min >= 12 || max >= 12) return 7;
				//if (onTimer) printf("cron_parser: monthsOn ");
				//else printf("cron_parser: monthsOff ");
				for (k = min; k <= max; k++) {
					if (onTimer) monthsOn[k] = 1;
					else monthsOff[k] = 1;
					//printf("%d ", k);
				}
				//printf("\n");
				break;
			case 5: //day-of-week
				if (min == 255) { min = 0; max = 6; }
				if (min >= 7 || max >= 7) return 8;
				//if (onTimer) printf("cron_parser: dweekOn ");
				//else printf("cron_parser: dweekOff ");
				for (k = min; k <= max; k++) {
					if (onTimer) dweekOn[k] = 1;
					else dweekOff[k] = 1;
					//printf("%d ", k);
				}
				//printf("\n");
				break;
			}
		}
	}
	return 0;
}
static int parse_body(char * body) {
	char * cronlineOn = NULL;
	char * cronlineOff = NULL;
	char * relay_task_interval_char = NULL;
	char * timezone = NULL;
	int e = 0;

	memset(secondsOn, 0, 60);
	memset(minutesOn, 0, 60);
	memset(hoursOn, 0, 24);
	memset(daysOn, 0, 31);
	memset(monthsOn, 0, 12);
	memset(secondsOff, 0, 60);
	memset(minutesOff, 0, 60);
	memset(hoursOff, 0, 24);
	memset(daysOff, 0, 31);
	memset(monthsOff, 0, 12);
	//	ESP_LOGI(TAG,"body:\n%s", body);

	cronlineOn = strtok(body, "\r\n");
	if (!cronlineOn) return 1;
	ESP_LOGI(TAG,"parse_body: cronlineOn %s", cronlineOn);
	cronlineOff = strtok(NULL, "\r\n");
	if (!cronlineOff) return 2;
	ESP_LOGI(TAG,"parse_body: cronlineOff %s", cronlineOff);
	relay_task_interval_char = strtok(NULL, "\r\n");
	if (!relay_task_interval_char) return 3;
	// ESP_LOGI(TAG,"relay_task_interval_char %s", relay_task_interval_char);
	if (isdigit((unsigned char)relay_task_interval_char[0])) {
		relay_task_interval = atoi(relay_task_interval_char);
		ESP_LOGI(TAG, "parse_body: relay_task_interval %u", relay_task_interval);
	}
	else return 4;
	timezone = strtok(NULL, "\r\n");
	if (!timezone) return 5;
	ESP_LOGI(TAG, "parse_body: timezone %s", timezone);
	tz = timezone;
	if (strncmp(tz, TIMEZONE, sizeof(TIMEZONE)) != 0) {
		ESP_LOGI(TAG, "parse_body: setting timezone %s...", tz);
		setenv("TZ", tz, 1);
		tzset();
	}

	e = cron_parser(cronlineOn, true);
	if (e != 0) ESP_LOGE(TAG, "parse_body: cron_parser(cronlineOn) error %u", e);
	e = cron_parser(cronlineOff, false);
	if (e != 0) ESP_LOGE(TAG, "parse_body: cron_parser(cronlineOff) error %u", e);
	return e;
}

static void http_update_task(void *arg)
{
	char buf[1024], * b;
	int s, r, t;
	const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
		.ai_protocol = IPPROTO_TCP,
	};
	struct addrinfo * res;
	uint32_t rand;

	while (1) {
		ESP_LOGI(TAG, "http_update_task: checking for an updated crontab...");
		b = buf;
		t = sizeof(buf);
		r = 0;
		s = -1;
		res = NULL;
		rand = esp_random(); //return uint32_t from 0..UINT32_MAX range
		//normalizing rand to be within 0..1000 range
		rand /= (UINT32_MAX - rand);
		rand = rand * 1000 / (1 + rand);

		int err = getaddrinfo(WEB_SERVER, WEB_SERVER_PORT, &hints, &res);
		if (err != 0) {
			ESP_LOGE(TAG, "nslookup_task: DNS lookup failed - %d", err);
			ESP_LOGW(TAG, "nslookup_task: out of precausion turning relay off...");
			gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			vTaskDelay(pdMS_TO_TICKS(rand));
			continue;
		}

		s = socket(res->ai_family, res->ai_socktype, 0);
		if (s < 0) {
			ESP_LOGE(TAG, "http_update_task: socket failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: allocated socket");

		if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
			ESP_LOGE(TAG, "http_update_task: connect failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: connected");

		struct timeval receiving_timeout;
		receiving_timeout.tv_sec = HTTP_TIMEOUT;
		receiving_timeout.tv_usec = 0;
		if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout, sizeof(receiving_timeout)) < 0) {
			ESP_LOGE(TAG, "http_update_task: failed to set socket receiving timeout");
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: set socket receiving timeout success");

		char buff[1024];
		int l = sprintf(buff, "%s %s %s\r\nHost: %s:%s\r\nConnection: close\r\nUser-Agent: %s\r\nAccept: %s\r\n\r\n", HTTP_METHOD, WEB_SERVER_PATH, HTTP_VERSION, WEB_SERVER, WEB_SERVER_PORT, USER_AGENT, "text/plain");
		//ESP_LOGI(TAG, "request: sending GET request:\n%s", buff);
		if (write(s, buff, l) < 0) {
			ESP_LOGE(TAG, "http_update_task: write failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: socket send success");

		memset(buf, 0, sizeof(buf));
		do {
			r = read(s, b, t);
			/*for (int i = 0; i < r; i++) {
				putchar(b[i]);
			}*/
			b += r;
			t = buf + sizeof(buf) - b;
			if (t <= 0) {
				ESP_LOGE(TAG, "http_update_task: receive buffer is full");
				ESP_LOGW(TAG, "http_update_task: out of precausion turning relay off...");
				gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
				goto sleep;
			}
		} while (r > 0);
		//ESP_LOGI(TAG, "http_update_task: done reading from socket. Last read return=%d errno=%d\n", r, errno);

		int http_status, content_len = 0, body_len, e;
		char * body, * cl = NULL;
	
		if (strncmp(buf, "HTTP/1.", 7) == 0) {
			http_status = atoi(buf + strlen(HTTP_VERSION) + 1);
			if (http_status == 200) {
				cl = strstr(buf, "Content-Length: ");
				if (cl) content_len = atoi(cl + 16);
				body = strstr(buf, "\r\n\r\n");
				if (body) body += 4;
				body_len = strlen(body);
				if (body_len == content_len && body_len != 0) {
					e = parse_body(body);
					if (e != 0) {
						ESP_LOGE(TAG, "http_update_task: parse_body returned error %d", e);
						ESP_LOGW(TAG, "http_update_task: out of precausion turning relay off...");
						gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
						goto sleep;
					}
				}
				else
				{
					ESP_LOGE(TAG, "http_update_task: content_len != body_len or body_len = 0");
					ESP_LOGW(TAG, "http_update_task: out of precausion turning relay off...");
					gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
					goto sleep;
				}
			}
			else {
				ESP_LOGE(TAG, "http_update_task: http_status %d", http_status);
				ESP_LOGW(TAG, "http_update_task: out of precausion turning relay off...");
				gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
				goto sleep;
			}
		}
		else {
			ESP_LOGE(TAG, "http_update_task: not an HTTP packet");
			ESP_LOGW(TAG, "http_update_task: out of precausion turning relay off...");
			gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			goto sleep;
		}

	sleep:
		if (res) freeaddrinfo(res);
		if (s >= 0) {
			shutdown(s, SHUT_RDWR);
			close(s);
		}
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);

		vTaskDelay(pdMS_TO_TICKS(HTTP_UPDATE_INTERVAL));
	}
}

static void relay_task(void *arg) {
	char datetime[64];
	char * datetime_field[4];//do not need year
	char * hms[3];
	bool on, off;
	int i, j;
	time_t now;
	struct tm timeinfo;
	TickType_t xLastWakeTime;

	while (1) {
		on = true;
		off = true;
		now = 0;
		memset(&timeinfo, 0, sizeof(timeinfo));
		time(&now);
		localtime_r(&now, &timeinfo);
		if (timeinfo.tm_year >= 120) { //we got the time
			//datetime Sat May 16 05:17:04 2020
			strftime(datetime, sizeof(datetime), "%a %b %d %H:%M:%S %Y", &timeinfo);
			ESP_LOGI(TAG, "datetime %s", datetime);

			i = 0;
			datetime_field[i] = strtok(datetime, " ");
			//ESP_LOGI(TAG, "relay_task: datetime_field[%d] %s", i, datetime_field[i]);
			while (datetime_field[i] && i < 3) {
				i++;
				datetime_field[i] = strtok(NULL, " ");
				//ESP_LOGI(TAG, "relay_task: datetime_field[%d] %s", i, datetime_field[i]);
			}
			i = 0;
			if (!datetime_field[3]) {
				ESP_LOGE(TAG, "relay_task: hms is NULL - datetime is invalid");
				goto sleep;
			}
			hms[i] = strtok(datetime_field[3], ":");
			//ESP_LOGI(TAG, "relay_task: hms[%d] %s", i, hms[i]);
			while (hms[i] && i < 2) {
				i++;
				hms[i] = strtok(NULL, ":");
				//ESP_LOGI(TAG, "relay_task: hms[%d] %s", i, hms[i]);
			}
			for (i = 0; i <= 3; i++) {
				switch (i) {
				case 0: //day-of-the-week
					if (!datetime_field[i]) {
						ESP_LOGE(TAG, "relay_task: day-of-week is NULL");
						goto sleep;
					}
					if (strncmp(datetime_field[i], "Sun", 3) == 0) j = 0;
					else if (strncmp(datetime_field[i], "Mon", 3) == 0) j = 1;
					else if (strncmp(datetime_field[i], "Tue", 3) == 0) j = 2;
					else if (strncmp(datetime_field[i], "Wed", 3) == 0) j = 3;
					else if (strncmp(datetime_field[i], "Thu", 3) == 0) j = 4;
					else if (strncmp(datetime_field[i], "Fri", 3) == 0) j = 5;
					else if (strncmp(datetime_field[i], "Sat", 3) == 0) j = 6;
					else {
						ESP_LOGE(TAG, "relay_task: invalid day-of-week");
						goto sleep;
					}
					//ESP_LOGI(TAG, "relay_task: day-of-week %d", j);
					if (!dweekOn[j]) {
						on = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOn line");
					}
					if (!dweekOff[j]) {
						off = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOff line");
					}
					break;
				case 1: //Month
					if (!datetime_field[i]) {
						ESP_LOGE(TAG, "relay_task: month is NULL");
						goto sleep;
					}
					if (strncmp(datetime_field[i], "Jan", 3) == 0) j = 0;
					else if (strncmp(datetime_field[i], "Feb", 3) == 0) j = 1;
					else if (strncmp(datetime_field[i], "Mar", 3) == 0) j = 2;
					else if (strncmp(datetime_field[i], "Apr", 3) == 0) j = 3;
					else if (strncmp(datetime_field[i], "May", 3) == 0) j = 4;
					else if (strncmp(datetime_field[i], "Jun", 3) == 0) j = 5;
					else if (strncmp(datetime_field[i], "Jul", 3) == 0) j = 6;
					else if (strncmp(datetime_field[i], "Aug", 3) == 0) j = 7;
					else if (strncmp(datetime_field[i], "Sep", 3) == 0) j = 8;
					else if (strncmp(datetime_field[i], "Oct", 3) == 0) j = 9;
					else if (strncmp(datetime_field[i], "Nov", 3) == 0) j = 10;
					else if (strncmp(datetime_field[i], "Dec", 3) == 0) j = 11;
					else {
						ESP_LOGE(TAG, "relay_task: invalid month");
						goto sleep;
					}
					//ESP_LOGI(TAG, "relay_task: month %d", j);
					if (!monthsOn[j]) {
						on = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOn line");
					}
					if (!monthsOff[j]) {
						off = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOff line");
					}
					break;
				case 2: //day
					if (!datetime_field[i]) {
						ESP_LOGE(TAG, "relay_task: day is NULL");
						goto sleep;
					}
					if (isdigit((unsigned char)datetime_field[i][0]))
						j = atoi(datetime_field[i]) - 1;
					else {
						ESP_LOGE(TAG, "relay_task: days is not a number");
						goto sleep;
					}
					if (j > 30) {
						ESP_LOGE(TAG, "relay_task: days are from 1 to 31");
						goto sleep;
					}
					//ESP_LOGI(TAG, "relay_task: day %d", j);
					if (!daysOn[j]) {
						on = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOn line");
					}
					if (!daysOff[j]) {
						off = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOff line");
					}
					break;
				case 3: //hms
					//hour
					if (!hms[0]) {
						ESP_LOGE(TAG, "relay_task: hours is NULL");
						goto sleep;
					}
					if (isdigit((unsigned char)hms[0][0]))
						j = atoi(hms[0]);
					else {
						ESP_LOGE(TAG, "relay_task: hours is not a number");
						goto sleep;
					}
					if (j > 23) {
						ESP_LOGE(TAG, "relay_task: hours are from 0 to 23");
						goto sleep;
					}
					//ESP_LOGI(TAG, "relay_task: hour %d", j);
					if (!hoursOn[j]) {
						on = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOn line");
					}
					if (!hoursOff[j]) {
						off = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOff line");
					}

					//minute
					if (!hms[1]) {
						ESP_LOGE(TAG, "relay_task: minutes is NULL");
						goto sleep;
					}
					if (isdigit((unsigned char)hms[1][0]))
						j = atoi(hms[1]);
					else {
						ESP_LOGE(TAG, "relay_task: minutes is not a number");
						goto sleep;
					}
					if (j > 59) {
						ESP_LOGE(TAG, "relay_task: minutes are from 0 to 59");
						goto sleep;
					}
					//ESP_LOGI(TAG, "relay_task: minute %d", j);
					if (!minutesOn[j]) {
						on = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOn line");
					}
					if (!minutesOff[j]) {
						off = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOff line");
					}

					//seconds
					if (!hms[2]) {
						ESP_LOGE(TAG, "relay_task: seconds is NULL");
						goto sleep;
					}
					if (isdigit((unsigned char)hms[2][0]))
						j = atoi(hms[2]);
					else {
						ESP_LOGE(TAG, "relay_task: seconds is not a number");
						goto sleep;
					}
					if (j > 59) {
						ESP_LOGE(TAG, "relay_task: seconds are from 0 to 59");
						goto sleep;
					}
					//ESP_LOGI(TAG, "relay_task: second %d", j);
					if (!secondsOn[j]) {
						on = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOn line");
					}
					if (!secondsOff[j]) {
						off = false;
						//ESP_LOGI(TAG, "relay_task: not in crontabOff line");
					}
					break;
				}
				//ESP_LOGI(TAG, "relay_task: on is %u, off is %u, i = %d", on, off, i);
				if (!on && !off) break;
			}
			if (on) {
				ESP_LOGI(TAG, "relay_task: turning relay on...");
				gpio_set_level(GPIO_NUM_0, 0); //Set GPIO0 as low - level output.
			}
			else if (off) {
				ESP_LOGI(TAG, "relay_task: turning relay off...");
				gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			}
			else {
				ESP_LOGI(TAG, "relay_task: the current time does not match any of the cronlines");
			}
		}
		else {
			ESP_LOGE(TAG, "relay_task: unable to get time from sntp server");
			ESP_LOGW(TAG, "relay_task: out of precausion turning relay off...");
			gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			goto sleep;
		}
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);

	sleep:
		xLastWakeTime = xTaskGetTickCount();
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(relay_task_interval));
	}
}

static void fw_update_task(void *arg) {
	int http_status, s, r;
	esp_err_t e;
	uint16_t rand;
	char buf[1460];
	char * body;
	const esp_partition_t * upgrade_partition;
	esp_ota_handle_t ota_handle = NULL;
	const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
		.ai_protocol = IPPROTO_TCP,
	};
	struct addrinfo * res;

	while (1) {
		ESP_LOGI(TAG, "fw_update_task: checking for a new timer image %s...", FW_PATH);
		s = -1;
		res = NULL;
		rand = esp_random(); //return uint32_t from 0..UINT32_MAX range
		//normalizing rand to be within 0..1000 range
		rand /= (UINT32_MAX - rand);
		rand = rand * 1000 / (1 + rand);

		int err = getaddrinfo(WEB_SERVER, WEB_SERVER_PORT, &hints, &res);
		if (err != 0) {
			ESP_LOGE(TAG, "nslookup_task: DNS lookup failed - %d", err);
			ESP_LOGW(TAG, "nslookup_task: out of precausion turning relay off...");
			gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			vTaskDelay(pdMS_TO_TICKS(rand));
			continue;
		}

		s = socket(res->ai_family, res->ai_socktype, 0);
		if (s < 0) {
			ESP_LOGE(TAG, "fw_update_task: socket failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: allocated socket");

		if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
			ESP_LOGE(TAG, "fw_update_task: connect failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: connected");

		struct timeval receiving_timeout;
		receiving_timeout.tv_sec = HTTP_TIMEOUT;
		receiving_timeout.tv_usec = 0;
		if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout, sizeof(receiving_timeout)) < 0) {
			ESP_LOGE(TAG, "fw_update_task: failed to set socket receiving timeout");
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: set socket receiving timeout success");

		char buff[1024];
		int l = sprintf(buff, "%s %s %s\r\nHost: %s:%s\r\nConnection: close\r\nUser-Agent: %s\r\nAccept: %s\r\n\r\n", HTTP_METHOD, FW_PATH, HTTP_VERSION, WEB_SERVER, WEB_SERVER_PORT, USER_AGENT, "application/octet-stream");
		//ESP_LOGI(TAG, "request: sending GET request:\n%s", buff);
		if (write(s, buff, l) < 0) {
			ESP_LOGE(TAG, "fw_update_task: write failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: socket send success");

		memset(buf, 0, sizeof(buf));
		r = read(s, buf, sizeof(buf));

		if (strncmp(buf, "HTTP/1.", 7) == 0) {
			http_status = atoi(buf + strlen(HTTP_VERSION) + 1);
			if (http_status == 200) {
				upgrade_partition = esp_ota_get_next_update_partition(NULL);
				if (upgrade_partition) {
					e = esp_ota_begin(upgrade_partition, OTA_SIZE_UNKNOWN, &ota_handle);
					if (e != ESP_OK) {
						ESP_LOGE(TAG, "fw_update_task: esp_ota_begin failed with error %u", e);
						goto sleep;
					}
					body = strstr(buf, "\r\n\r\n");
					if (body) {
						body += 4;
						r -= (body - buf);
						if (r > 0) {
							e = esp_ota_write(ota_handle, body, r);
							if (e != ESP_OK) {
								ESP_LOGE(TAG, "fw_update_task: esp_ota_write failed with error %u", e);
								goto sleep;
							}
						}
					}
				}
				else {
					ESP_LOGE(TAG, "fw_update_task: unable to get next update partition");
					goto sleep;
				}
			}
			else if (http_status == 404) {
				ESP_LOGI(TAG, "fw_update_task: no new timer image available. Next check is in %u ms", FW_CHECK_INTERVAL);
				goto sleep;
			}
			else {
				ESP_LOGE(TAG, "fw_update_task: http_status %d", http_status);
				goto sleep;
			}
		}
		else {
			ESP_LOGE(TAG, "fw_update_task: not an HTTP packet");
			goto sleep;
		}

		do {
			memset(buf, 0, sizeof(buf));
			r = read(s, buf, sizeof(buf));
			e = esp_ota_write(ota_handle, buf, r);
			if (e != ESP_OK) {
				ESP_LOGE(TAG, "fw_update_task: esp_ota_write failed with error %u", e);
				goto sleep;
			}
		} while (r > 0);
		//ESP_LOGI(TAG, "fw_update_task: done reading from socket. Last read return=%d errno=%d\n", r, errno);		
		
		e = esp_ota_end(ota_handle);
		if (e != ESP_OK) {
			ESP_LOGE(TAG, "fw_update_task: esp_ota_end failed with error %u", e);
			goto sleep;
		}
		ESP_LOGI(TAG, "fw_update_task: done uploading the new image. Rebooting...\n");
		e = esp_ota_set_boot_partition(upgrade_partition);
		if (e != ESP_OK) {
			ESP_LOGE(TAG, "fw_update_task: esp_ota_set_boot_partition failed with error %u", e);
			goto sleep;
		}
		esp_restart();

	sleep:
		if (res) freeaddrinfo(res);
		if (s >= 0) {
			shutdown(s, SHUT_RDWR);
			close(s);
		}
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);

		vTaskDelay(pdMS_TO_TICKS(FW_CHECK_INTERVAL));
	}
}

static void esp_config_task(void *arg)
{
	esp_err_t err;
	uint32_t notificationValue;

	while (1) {
		notificationValue = ulTaskNotifyTake(1, pdMS_TO_TICKS(ESP_CONFIG_TASK_WAIT_NOTIFY_MS));
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
			ESP_LOGI(TAG, "dns 1: %s", ipaddr_ntoa(&prim_dns_ip.ip));
			ESP_LOGI(TAG, "dns 2: %s", ipaddr_ntoa(&sec_dns_ip.ip));

			ESP_LOGI(TAG, "esp_config_task: initializing SNTP...");
			sntp_setoperatingmode(SNTP_OPMODE_POLL);
			sntp_setservername(0, NTP0);
			sntp_setservername(1, NTP1);
			sntp_setservername(2, NTP2);
			sntp_init();

			ESP_LOGI(TAG, "esp_config_task: setting timezone %s...", tz);
			setenv("TZ", tz, 1);
			tzset();

			ESP_LOGI(TAG, "sntp 0: %s", sntp_getservername(0));
			ESP_LOGI(TAG, "sntp 1: %s", sntp_getservername(1));
			ESP_LOGI(TAG, "sntp 2: %s", sntp_getservername(2));

			char par[strlen(TIMER_ON_CRONLINE) + strlen(TIMER_OFF_CRONLINE) + 15];
			sprintf(par, "%s\n%s\n%u\n%s", TIMER_ON_CRONLINE, TIMER_OFF_CRONLINE, RELAY_TASK_INTERVAL, TIMEZONE);
			parse_body(par);

			if (HTTP_UPDATE)
				if (pdPASS != xTaskCreate(&http_update_task, "http_update_task", HTTP_UPDATE_TASK_SS, NULL, HTTP_UPDATE_TASK_PRIORITY, NULL))
					ESP_LOGE(TAG, "failed to create http update task");
			if (pdPASS != xTaskCreate(&relay_task, "relay_task", RELAY_TASK_SS, NULL, RELAY_TASK_PRIORITY, NULL))
				ESP_LOGE(TAG, "failed to create relay task");
			if (HTTP_FW_UPDATE)
				if (pdPASS != xTaskCreate(&fw_update_task, "fw_update_task", FW_UPDATE_TASK_SS, NULL, FW_UPDATE_TASK_PRIORITY, NULL))
					ESP_LOGE(TAG, "failed to create fw update task");

			ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT));
		}
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
	}
}

static esp_err_t event_handler(void *ctx, system_event_t *evt)
{
	//ESP_LOGI(TAG,"event %x", evt->event_id);
	char * reason = "";
	esp_err_t err;
	wifi_ap_record_t ap_records[MAX_AP_RECORDS];
	uint16_t ap_record_number = MAX_AP_RECORDS, ap_number, i;

	switch (evt->event_id) {
	case SYSTEM_EVENT_WIFI_READY: //should be triggered after esp-wifi_init() but it is not!
		ESP_LOGI(TAG, "event_handler: wifi is ready");
		break;
	case SYSTEM_EVENT_SCAN_DONE:
		err = esp_wifi_scan_get_ap_num(&ap_number);
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_scan_get_ap_num failed - %s", esp_err_to_name(err));
		//ESP_LOGI(TAG, "ap scan done, status %u, number of APs %u", evt->event_info.scan_done.status, ap_number);
		//esp_wifi_scan_get_ap_records call is needed to free memory allocated by esp_wifi_scan_start
		err = esp_wifi_scan_get_ap_records(&ap_record_number, ap_records);
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_scan_get_ap_records failed - %s", esp_err_to_name(err));
		ESP_LOGI(TAG, "Found %u APs:", ap_record_number);
		for (i = 0; i < ap_record_number; i++) {
			if (i >= MAX_AP_RECORDS) {
				ESP_LOGE(TAG, "number of found APs %u exceeds the maximum %u", ap_record_number, MAX_AP_RECORDS);
				break;
			}
			ap_records[i].ssid[32] = '\0';
			ESP_LOGI(TAG, "%s (%x:%x:%x:%x:%x:%x) %udBm?, %s, 11b: %u, 11g: %u, 11n: %u, low_rate: %u", ap_records[i].ssid, ap_records[i].bssid[0], ap_records[i].bssid[1], ap_records[i].bssid[2], ap_records[i].bssid[3], ap_records[i].bssid[4], ap_records[i].bssid[5],
				ap_records[i].rssi, authmode(ap_records[i].authmode), ap_records[i].phy_11b, ap_records[i].phy_11g, ap_records[i].phy_11n, ap_records[i].phy_lr);
		}
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

		//err = esp_wifi_scan_stop();
		//err = esp_wifi_scan_start(&config, false); //scan is done as part of esp_wifi_connect()

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
		ESP_LOGW(TAG, "event_handler: disconnected from ssid %s, reason %s, re-scanning...",
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
		ESP_LOGI(TAG, "event_handler: station got ip:" IPSTR ", mask:" IPSTR ", gw:" IPSTR,
			IP2STR(&evt->event_info.got_ip.ip_info.ip),
			IP2STR(&evt->event_info.got_ip.ip_info.netmask),
			IP2STR(&evt->event_info.got_ip.ip_info.gw));
		if (!USE_STATIC_IP) {
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

static void sendlog() {
	int	err;
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock >= 0) {
		err = sendto(sock, log_buffer, log_buffer_pointer, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
		if (err < 0) {
			printf("sendlog: sendto failed - %s\n", strerror(errno));
		}
		else {
			memset(log_buffer, 0, MAX_LOG_BUFFER_SIZE);
			log_buffer_pointer = 0;
		}
		close(sock);
	}
	else printf("sendlog: socket failed - %s, remote logging will not work\n", strerror(errno));
}

static void remote_logging_task(void *arg) {
	uint32_t notificationValue;

	remote_addr.sin_family = AF_INET;
	remote_addr.sin_addr.s_addr = inet_addr(REMOTE_LOGGING_IP);
	remote_addr.sin_port = htons(REMOTE_LOGGING_UDP_PORT);

	while (1) {
		notificationValue = ulTaskNotifyTake(1, pdMS_TO_TICKS(CHAR_TIMEOUT));
		if (notificationValue)
			sendlog();
		else if (log_buffer_pointer > 0)
			sendlog();
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
	}
}

static int remote_logging(int ch) {
	if (log_buffer_pointer == MAX_LOG_BUFFER_SIZE) {
		if (logging_task_handle)
			xTaskNotifyGive(logging_task_handle);
		else printf("remote_logging: logging_task_handle is NULL\n");
		putchar(ch);
	}
	else {
		log_buffer[log_buffer_pointer++] = ch;
		if (log_buffer_pointer == MAX_LOG_BUFFER_SIZE) {
			if (logging_task_handle)
				xTaskNotifyGive(logging_task_handle);
			else printf("remote_logging: logging_task_handle is NULL\n");
		}
	}
	return ch;
}

void app_main(void) {
	esp_err_t err;
	if (REMOTE_LOGGING) {
		memset(log_buffer, 0, MAX_LOG_BUFFER_SIZE);
		log_buffer_pointer = 0;
		esp_log_set_putchar(remote_logging);
		if (pdPASS != xTaskCreate(&remote_logging_task, "logging_task", REMOTE_LOGGING_TASK_SS, NULL, LOGGING_TASK_PRIORITY, &logging_task_handle))
			printf("app_main: failed to create logging task\n");
	}

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

	//ESP_LOGI(TAG, "setting wifi config storage to RAM...");
	//err = esp_wifi_set_storage(WIFI_STORAGE_RAM); // WIFI_STORAGE_FLASH - default
	//if (err != ESP_OK)
	//	ESP_LOGE(TAG, "app_main: esp_wifi_set_storage failed - %s", esp_err_to_name(err));
	//ESP_LOGI(TAG, "setting esp to station mode...");
	err = esp_wifi_set_mode(WIFI_MODE_STA);
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_set_mode failed - %s", esp_err_to_name(err));
	//err = esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT20); //WIFI_BW_HT40 for 802.11n
	//err = esp_wifi_set_auto_connect(false); // default is true - deprecated!

	if (pdPASS != xTaskCreate(&esp_config_task, "esp_config_task", ESP_CONFIG_TASK_SS, NULL, 5, &esp_config_task_handle))
		ESP_LOGE(TAG, "app_main: failed to create esp config task");

	err = esp_wifi_start();
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_start failed - %s", esp_err_to_name(err));

	/* TO DO - create a task to monitor wifi state
	wifi_state_t state = esp_wifi_get_state();
	switch (state) {
	case WIFI_STATE_DEINIT:
		break;
	case WIFI_STATE_INIT:
		break;
	case WIFI_STATE_START:
		break;
	default:
	}
	*/
}
