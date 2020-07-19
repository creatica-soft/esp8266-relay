/*
=== Relay controlled by SNTP timer (RTOS SDK based) ===

Install toolchain for crosscompiling (may not be easy, old esp-open-sdk toolchain works fine despite of warnings):
 - supported toolchain version is 1.22.0-92-g8facf4c
 - supported compiler version 5.2.0

 export PATH=~/esp-open-sdk/xtensa-lx106-elf/bin:$PATH

cd ~
wget https://github.com/espressif/ESP8266_RTOS_SDK/archive/v3.1.2.tar.gz
tar -zxf ESP8266_RTOS_SDK-v3.1.2.tar.gz
mkdir -p ESP8266_RTOS_SDK-3.1.2/timer/main

Review and update defines in user_main.c

cp timer.c ESP8266_RTOS_SDK-3.1.2/timer/main
cp ESP8266_RTOS_SDK-3.1.2/examples/get-started/project-template/Makefile ESP8266_RTOS_SDK-3.1.2/timer
touch ESP8266_RTOS_SDK-3.1.2/timer/main/component.mk

export IDF_PATH=~/ESP8266_RTOS_SDK-3.1.2

make menuconfig # to produce sdkconfig

make

Power off, ground GPIO0 before flashing, power on

make flash

Power off, unground GPIO0 for normal operation, power on

Run terminal emulator to see the ESP_LOGI(TAG,) messages:
miniterm --raw /dev/ttyAMA1 74880

I (37) boot: ESP-IDF  2nd stage bootloader
I (37) boot: compile time 12:00:41
I (43) qio_mode: Enabling default flash chip QIO
I (43) boot: SPI Speed      : 40MHz
I (45) boot: SPI Mode       : QIO
I (49) boot: SPI Flash Size : 1MB
I (53) boot: Partition Table:
I (57) boot: ## Label            Usage          Type ST Offset   Length
I (64) boot:  0 nvs              WiFi data        01 02 00009000 00004000
I (71) boot:  1 otadata          OTA data         01 00 0000d000 00002000
I (79) boot:  2 phy_init         RF data          01 01 0000f000 00001000
I (86) boot:  3 ota_0            OTA app          00 10 00010000 00070000
I (94) boot:  4 ota_1            OTA app          00 11 00080000 00070000
I (101) boot: End of partition table
E (105) boot: ota data partition invalid and no factory, will try all partitions
I (114) esp_image: segment 0: paddr=0x00010010 vaddr=0x40210010 size=0x45138 (282936) map
I (171) esp_image: segment 1: paddr=0x00055150 vaddr=0x3ffe8000 size=0x00678 (  1656) load
I (171) esp_image: segment 2: paddr=0x000557d0 vaddr=0x3ffe8678 size=0x00110 (   272) load
I (178) esp_image: segment 3: paddr=0x000558e8 vaddr=0x40100000 size=0x073d4 ( 29652) load
I (192) boot: Loaded app from partition at offset 0x10000
I (219) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (219) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (289) reset_reason: RTC reset 2 wakeup 0 store 4213622506, reason is 2
unsupport wifi protocol bitmap
I (309) timer: wifi set 802.11g (<54Mbps, 38-140m) mode ok
I (309) timer: Setting esp to station mode...
I (319) timer: Configuring esp for scan method 0, channel 1...
I (319) timer: Connecting to ssid
*/

//#define NDEBUG //to get rid of ESP_ERROR_CHECK assert messages

#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <ctype.h>
#include <netdb.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event_loop.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "rom/ets_sys.h"
#include "tcpip_adapter.h"
#include "lwip/netif.h"
#include "lwip/netdb.h"
#include "lwip/dhcp.h"
#include "lwip/apps/sntp.h"
#include "driver/gpio.h"
#include "esp_ota_ops.h"

#define SSID "ssid"
#define PASSPHRASE "password"
//iw wlan0 station dump to check the signal >= -50dBm is good 
// WIFI_POWER values, level0 - is the maximum power
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
#define WIFI_POWER 7
//wifi modes: //WIFI_PROTOCOL_11B, WIFI_PROTOCOL_11G, WIFI_PROTOCOL_11N, can be ORed (|)
#define WIFI_MODE WIFI_PROTOCOL_11G
#define SSID_CHANNEL 1 //optional channel for AP scan
#define MIN_SCANTIME 1000 //wifi min scan time in ms
#define MAX_SCANTIME 5000 //wifi max scan time in ms
#define WAIT_FOR_WIFI_RETRY_MS 10000 //retry waiting for wifi connection after 10 sec
#define HOSTNAME "timer"
#define NTP0 "0.pool.ntp.org"
#define NTP1 "1.pool.ntp.org"
#define NTP2 "2.pool.ntp.org"
#define DNS1 "192.168.1.1"
#define DNS2 "8.8.8.8"

//update timer parameters from http://WEB_SERVER:WEB_SERVER_PORT/WEB_SERVER_PATH
#define HTTP_UPDATE true

//check for new timer parameters every ms
#define HTTP_UPDATE_INTERVAL 600000
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
#define USE_STATIC_IP false
#define STATIC_IP "192.168.1.101"
#define NETMASK "255.255.255.0"
#define GATEWAY_IP "192.168.1.1"

unsigned int relay_task_interval = RELAY_TASK_INTERVAL;
char * tz = TIMEZONE;
int secondsOn[60], minutesOn[60], hoursOn[24], daysOn[31], monthsOn[12], dweekOn[7];
int secondsOff[60], minutesOff[60], hoursOff[24], daysOff[31], monthsOff[12], dweekOff[7];

static const char *TAG = "timer";
const int CONNECTED_BIT = BIT0;
static EventGroupHandle_t wifi_event_group;

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

static int request(const char * media_type, const char * request_file) {
	const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct addrinfo *res;
	struct in_addr *addr;
	int s;

	while (1) {
		int err = getaddrinfo(WEB_SERVER, WEB_SERVER_PORT, &hints, &res);

		if (err != 0 || res == NULL) {
			ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
			vTaskDelay(pdMS_TO_TICKS(1000));
			ESP_LOGI(TAG, "http_update_task: out of precausion turning relay off...");
			gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			continue;
		}
		addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
		ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

		s = socket(res->ai_family, res->ai_socktype, 0);
		if (s < 0) {
			ESP_LOGE(TAG, "... Failed to allocate socket, error %d", errno);
			goto sleep;
		}
		ESP_LOGI(TAG, "... allocated socket");

		if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
			ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
			goto sleep;
		}

		ESP_LOGI(TAG, "... connected");

		char buff[1024];
		int l = sprintf(buff, "%s %s %s\r\nHost: %s:%s\r\nConnection: close\r\nUser-Agent: %s\r\nAccept: %s\r\n\r\n", HTTP_METHOD, request_file, HTTP_VERSION, WEB_SERVER, WEB_SERVER_PORT, USER_AGENT, media_type);
		ESP_LOGI(TAG, "sending GET request:\n%s", buff);

		if (write(s, buff, l) < 0) {
			ESP_LOGE(TAG, "... socket send failed");
			goto sleep;
		}
		ESP_LOGI(TAG, "... socket send success");

		struct timeval receiving_timeout;
		receiving_timeout.tv_sec = HTTP_TIMEOUT;
		receiving_timeout.tv_usec = 0;
		if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout, sizeof(receiving_timeout)) < 0) {
			ESP_LOGE(TAG, "... failed to set socket receiving timeout");
			goto sleep;
		}
		ESP_LOGI(TAG, "... set socket receiving timeout success");
		freeaddrinfo(res); 
		break;

	sleep:
		ESP_LOGI(TAG, "http_update_task: out of precausion turning relay off...");
		gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
		freeaddrinfo(res);
		if (s >= 0) close(s);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	return s;
}

static void http_update_task(void *pvParameters)
{
	char buf[1024], * b;
	int s, r, t;

	while (1) {
		b = buf;
		t = sizeof(buf);
		r = 0;

		s = request("text/plain", WEB_SERVER_PATH);

		/* Read HTTP response, r will be -1 if socket times out and no data received */
		memset(buf, 0, sizeof(buf));
		do {
			r = read(s, b, t);
			for (int i = 0; i < r; i++) {
				putchar(b[i]);
			}
			b += r;
			t = buf + sizeof(buf) - b;
			if (t <= 0) {
				ESP_LOGE(TAG, "http_update_task: receive buffer is full");
				ESP_LOGI(TAG, "http_update_task: out of precausion turning relay off...");
				gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
				goto sleep;
			}
		} while (r > 0);
		ESP_LOGI(TAG, "http_update_task: done reading from socket. Last read return=%d errno=%d\n", r, errno);

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
						ESP_LOGI(TAG, "http_update_task: out of precausion turning relay off...");
						gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
						goto sleep;
					}
				}
				else
				{
					ESP_LOGE(TAG, "http_update_task: content_len != body_len or body_len = 0");
					ESP_LOGI(TAG, "http_update_task: out of precausion turning relay off...");
					gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
					goto sleep;
				}
			}
			else {
				ESP_LOGE(TAG, "http_update_task: http_status %d", http_status);
				ESP_LOGI(TAG, "http_update_task: out of precausion turning relay off...");
				gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
				goto sleep;
			}
		}
		else {
			ESP_LOGE(TAG, "http_update_task: not an HTTP packet");
			ESP_LOGI(TAG, "http_update_task: out of precausion turning relay off...");
			gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			goto sleep;
		}

	sleep:
		close(s);
		TickType_t xLastWakeTime = xTaskGetTickCount();
		const TickType_t xPeriod = pdMS_TO_TICKS(HTTP_UPDATE_INTERVAL);
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
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
	TickType_t xPeriod;

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
			ESP_LOGI(TAG, "relay_task: out of precausion turning relay off...");
			gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
			goto sleep;
		}

	sleep:
		xLastWakeTime = xTaskGetTickCount();
		xPeriod = pdMS_TO_TICKS(relay_task_interval);
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

static void fw_update_task(void *arg) {
	int s, r;
	char buf[1460];
	int http_status;
	char * body;
	const esp_partition_t * upgrade_partition;
	esp_ota_handle_t ota_handle = NULL;
	esp_err_t e;

	while (1) {
		ESP_LOGI(TAG, "fw_update_task: checking for new timer image %s...", FW_PATH);
		s = request("application/octet-stream", FW_PATH);
		
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
		ESP_LOGI(TAG, "fw_update_task: done reading from socket. Last read return=%d errno=%d\n", r, errno);		
		
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
		close(s);
		TickType_t xLastWakeTime = xTaskGetTickCount();
		const TickType_t xPeriod = pdMS_TO_TICKS(FW_CHECK_INTERVAL);
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

static void esp_config_task(void *arg)
{
	TickType_t xLastWakeTime;
	TickType_t xPeriod;

	while (1) {
		EventBits_t uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
		if (uxBits & CONNECTED_BIT) {
			ESP_LOGI(TAG, "station got IP");
			sysinfo();
			ESP_LOGI(TAG, "setting hostname %s...", HOSTNAME);
			ESP_ERROR_CHECK(tcpip_adapter_set_hostname(WIFI_IF_STA, HOSTNAME));

			if (USE_STATIC_IP) {
				ESP_LOGI(TAG, "stopping dhcp client...");
				ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(WIFI_IF_STA));

				tcpip_adapter_ip_info_t ip;
				ipaddr_aton(STATIC_IP, &ip.ip);
				ipaddr_aton(GATEWAY_IP, &ip.gw);
				ipaddr_aton(NETMASK, &ip.netmask);

				ESP_LOGI(TAG, "setting static ip %s...", ipaddr_ntoa(&ip.ip));
				ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(WIFI_IF_STA, &ip));
			}

			tcpip_adapter_dns_info_t prim_dns_ip;
			tcpip_adapter_dns_info_t sec_dns_ip;

			ipaddr_aton(DNS1, &prim_dns_ip.ip);
			ipaddr_aton(DNS2, &sec_dns_ip.ip);
			ESP_LOGI(TAG, "setting primary dns %s...", DNS1);
			ESP_ERROR_CHECK(tcpip_adapter_set_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &prim_dns_ip));
			ESP_LOGI(TAG, "setting secondary dns %s...", DNS2);
			ESP_ERROR_CHECK(tcpip_adapter_set_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_BACKUP, &sec_dns_ip));
			tcpip_adapter_get_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &prim_dns_ip);
			tcpip_adapter_get_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_BACKUP, &sec_dns_ip);
			ESP_LOGI(TAG, "dns 1: %s", ipaddr_ntoa(&prim_dns_ip.ip));
			ESP_LOGI(TAG, "dns 2: %s", ipaddr_ntoa(&sec_dns_ip.ip));

			ESP_LOGI(TAG, "Initializing SNTP...");
			sntp_setoperatingmode(SNTP_OPMODE_POLL);
			sntp_setservername(0, NTP0);
			sntp_setservername(1, NTP1);
			sntp_setservername(2, NTP2);
			sntp_init();

			ESP_LOGI(TAG, "setting timezone %s...", tz);
			setenv("TZ", tz, 1);
			tzset();

			ESP_LOGI(TAG, "sntp 0: %s", sntp_getservername(0));
			ESP_LOGI(TAG, "sntp 1: %s", sntp_getservername(1));
			ESP_LOGI(TAG, "sntp 2: %s", sntp_getservername(2));

			char par[strlen(TIMER_ON_CRONLINE) + strlen(TIMER_OFF_CRONLINE) + 15];
			sprintf(par, "%s\n%s\n%u\n%s", TIMER_ON_CRONLINE, TIMER_OFF_CRONLINE, RELAY_TASK_INTERVAL, TIMEZONE);
			parse_body(par);

			if (HTTP_UPDATE)
				if (pdPASS != xTaskCreate(&http_update_task, "http_update_task", 8192, NULL, HTTP_UPDATE_TASK_PRIORITY, NULL))
					ESP_LOGE(TAG, "failed to create http update task");
			if (pdPASS != xTaskCreate(&relay_task, "relay_task", 8192, NULL, RELAY_TASK_PRIORITY, NULL))
				ESP_LOGE(TAG, "failed to create relay task");
			if (HTTP_FW_UPDATE)
				if (pdPASS != xTaskCreate(&fw_update_task, "fw_update_task", 8192, NULL, FW_UPDATE_TASK_PRIORITY, NULL))
					ESP_LOGE(TAG, "failed to create fw update task");

			ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT));

			vTaskDelete(NULL);
			break;
		}
		else {
			ESP_LOGE(TAG, "Timeout connecting to WiFi... Will retry in %u sec", WAIT_FOR_WIFI_RETRY_MS);
			xLastWakeTime = xTaskGetTickCount();
			xPeriod = pdMS_TO_TICKS(WAIT_FOR_WIFI_RETRY_MS);
			vTaskDelayUntil(&xLastWakeTime, xPeriod);
		}
	}
}

static esp_err_t event_handler(void *ctx, system_event_t *evt)
{
	//ESP_LOGI(TAG,"event %x", evt->event_id);
	char * reason = "";
	char * mode = "";
	wifi_scan_config_t config;
	switch (evt->event_id) {
	case SYSTEM_EVENT_WIFI_READY:
		ESP_LOGI(TAG, "wifi is ready");
		break;
	case SYSTEM_EVENT_SCAN_DONE:
		ESP_LOGI(TAG, "ap scan done, status %u", evt->event_info.scan_done.status);
		ESP_LOGI(TAG, "connecting to ap...");
		ESP_ERROR_CHECK(esp_wifi_connect());
		break;
	case SYSTEM_EVENT_STA_START:
		ESP_LOGI(TAG, "wifi started");
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_STOP:
		ESP_LOGI(TAG, "wifi stopped");
		ESP_LOGI(TAG, "event_handler: out of precausion turning relay off...");
		gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
		switch (evt->event_info.connected.authmode) {
		case WIFI_AUTH_OPEN:
			mode = "open";
			break;
		case WIFI_AUTH_WEP:
			mode = "wep";
			break;
		case WIFI_AUTH_WPA_PSK:
			mode = "wpa_psk";
			break;
		case WIFI_AUTH_WPA2_PSK:
			mode = "wpa2_psk";
			break;
		case WIFI_AUTH_WPA_WPA2_PSK:
			mode = "wpa_wpa2_psk";
			break;
		case WIFI_AUTH_WPA2_ENTERPRISE:
			mode = "wpa2_enterprise";
			break;
		case WIFI_AUTH_MAX:
			mode = "max";
			break;
		default:
			mode = "unknown";
		}
		ESP_LOGI(TAG, "connected to ssid %s, channel %d, authmode %s",
			evt->event_info.connected.ssid,
			evt->event_info.connected.channel, mode);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
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
		ESP_LOGI(TAG, "disconnected from ssid %s, reason %s, re-scanning...",
			evt->event_info.disconnected.ssid, reason);
		ESP_LOGI(TAG, "event_handler: out of precausion turning relay off...");
		gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
		config.ssid = (uint8_t *)SSID;
		config.bssid = NULL;
#ifdef SSID_CHANNEL
		config.channel = SSID_CHANNEL;
#endif // SSID_CHANNEL
		config.show_hidden = 1;
		config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
		config.scan_time.active.min = MIN_SCANTIME;
		config.scan_time.active.max = MAX_SCANTIME;
		esp_wifi_scan_start(&config, false);
		break;
	case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
		ESP_LOGI(TAG, "mode: %d -> %d",
			evt->event_info.auth_change.old_mode,
			evt->event_info.auth_change.new_mode);
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(TAG, "got ip:" IPSTR ", mask:" IPSTR ", gw:" IPSTR,
			IP2STR(&evt->event_info.got_ip.ip_info.ip),
			IP2STR(&evt->event_info.got_ip.ip_info.netmask),
			IP2STR(&evt->event_info.got_ip.ip_info.gw));
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_LOST_IP:
		ESP_LOGE(TAG, "lost ip");
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		ESP_LOGI(TAG, "event_handler: out of precausion turning relay off...");
		gpio_set_level(GPIO_NUM_0, 1); //Set GPIO0 as high - level output.
		break;
	default:
		ESP_LOGI(TAG, "unknown event with id %x", evt->event_id);
		break;
	}
	return ESP_OK;
}

void app_main(void) {
	// Initialize NVS.
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
		// OTA app partition table has a smaller NVS partition size than the non-OTA
		// partition table. This size mismatch may cause NVS initialization to fail.
		// If this happens, we erase NVS partition and initialize NVS again.
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	uint8_t wifi_mode = WIFI_MODE;
	if (esp_wifi_set_protocol(ESP_IF_WIFI_STA, wifi_mode)) {
		switch (wifi_mode) {
		case WIFI_PROTOCOL_11B:
			ESP_LOGI(TAG, "wifi set 802.11b (<11Mbps, 35-140m) mode ok");
			break;
		case WIFI_PROTOCOL_11G:
			ESP_LOGI(TAG, "wifi set 802.11g (<54Mbps, 38-140m) mode ok");
			break;
		case WIFI_PROTOCOL_11N:
			ESP_LOGI(TAG, "wifi set 802.11n (<289/600Mbps, 70-250m) mode ok");
			break;
		default:
			ESP_LOGI(TAG, "wifi mode is unknown %u", wifi_mode);
		}
	}
	else ESP_LOGI(TAG, "esp_wifi_set_protocol %d failed (1: b, 2: g, 4: n)", wifi_mode);
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
	ESP_LOGI(TAG, "Setting esp to station mode...");
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_LOGI(TAG, "Configuring esp for scan method %u, channel %u...", wifi_config.sta.scan_method, wifi_config.sta.channel);
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_LOGI(TAG, "Connecting to ssid %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_POWER));

	xTaskCreate(&esp_config_task, "esp_config_task", 8192, NULL, 5, NULL);
}
