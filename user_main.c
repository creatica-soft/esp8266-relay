/*
=== SNTP-based timer-relay ===
Review and update defines

wget https://github.com/espressif/ESP8266_NONOS_SDK/archive/v3.0.3.tar.gz
tar -zxf ESP8266_NONOS_SDK-v3.0.3.tar.gz
mkdir -p ESP8266_NONOS_SDK-3.0.3/timer/user
cp user_main.c ESP8266_NONOS_SDK-3.0.3/timer/user
cp ESP8266_NONOS_SDK-3.0.3/examples/IoT_Demo/Makefile ESP8266_NONOS_SDK-3.0.3/timer
Comment out "driver" lines in Makefile above
cp -r ESP8266_NONOS_SDK-3.0.3/examples/IoT_Demo/include ESP8266_NONOS_SDK-3.0.3/timer/
cp SP8266_NONOS_SDK-3.0.3/examples/IoT_Demo/user/Makefile ESP8266_NONOS_SDK-3.0.3/timer/user 

Install esp-open-sdk first for crosscompiling!

export PATH=~/esp-open-sdk/xtensa-lx106-elf/bin:$PATH

Compile user1.bin for esp-01s with "make COMPILE=gcc BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2"
Compile user2.bin for esp-01s with "make clean; make COMPILE=gcc BOOT=new APP=2 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2"

Install esptool from https://github.com/espressif/esptool

Power off, ground GPIO0 before flashing, power on

Power cycle after each flash!

Flash boot.bin first:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x00000 ../ESP8266_NONOS_SDK-3.0.3/bin/boot_v1.7.bin
Flash user1.bin:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x01000 ../ESP8266_NONOS_SDK-3.0.3/bin/upgrade/user1.1024.new.2.bin
Flash redundant image user2.bin:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x81000 ../ESP8266_NONOS_SDK-3.0.3/bin/upgrade/user2.1024.new.2.bin
Flash blank.bin and esp_init_data_default.bin:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfb000 ../ESP8266_NONOS_SDK-3.0.3/bin/blank.bin
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfc000 ../ESP8266_NONOS_SDK-3.0.3/bin/esp_init_data_default_v08.bin
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfe000 ../ESP8266_NONOS_SDK-3.0.3/bin/blank.bin

Power off, unground GPIO0 for normal operation, power on

Run terminal emulator to see the os_printf() messages:
miniterm --raw /dev/ttyAMA0 74880

2nd boot version : 1.5
SPI Speed      : 40MHz
SPI Mode       : QIO
SPI Flash Size & Map: 8Mbit(512KB+512KB)
jump to run user1 @ 1000

*/
#include "ets_sys.h"
#include "sntp.h"
#include "osapi.h"
#include "user_interface.h"
#include "gpio.h"
#include "ip_addr.h"
#include "mem.h"
#include "espconn.h"
#include "upgrade.h"

#define SSID "ssid"
#define PASSPHRASE "password"
//iw wlan0 station dump to check the signal >= -50dBm is good 
#define WIFI_POWER 10 // in 0.25dBm units from 0 to 82
#define SSID_CHANNEL 1 //optional channel for AP scan
#define MIN_SCANTIME 1000 //wifi min scan time in ms
#define MAX_SCANTIME 5000 //wifi max scan time in ms
#define HOSTNAME "relay"
#define NTP0 "0.pool.ntp.org"
#define NTP1 "1.pool.ntp.org"
#define NTP2 "2.pool.ntp.org"
#define DNS "192.168.1.1"

//update timer parameters from http://WEB_SERVER:WEB_SERVER_PORT/WEB_SERVER_PATH
#define HTTP_UPDATE false

//check for new timer parameters every ms
#define HTTP_UPDATE_INTERVAL 300000 
#define WEB_SERVER "example.com"
#define WEB_SERVER_PORT "80"

#define TIMER_ON_CRONLINE "0-4,20-24,40-44 * * * * *" //sec min hour day month day-of-week
#define TIMER_OFF_CRONLINE "10-14,30-24,50-54 * * * * *" //sec min hour day month day-of-week
//timezone from -11 to 13
#define RELAY_ARM_INTERVAL 5000 //period in ms to check if relay should be on or off
								//set it based on relay schedule
#define TIMEZONE 0

/*WEB_SERVER_PATH file should have four lines:
CRONLINE_ON
CRONLINE_OFF
RELAY_ARM_INTERVAL
TIMEZONE
*/
#define WEB_SERVER_PATH "/relay-timer-parameters.txt"
#define HTTP_METHOD "GET"
#define HTTP_VERSION "HTTP/1.1"
#define USER_AGENT "esp8266"

//Station static IP config
#define USE_STATIC_IP false
#define STATIC_IP "192.168.1.100"
#define NETMASK "255.255.255.0"
#define GATEWAY_IP "192.168.1.1"

#define ESP_CONFIG_ARM_INTERVAL 5000 //in ms
#define USER1_BIN_ADDRESS 0x01000 //SPI flash start address of user1.bin

#if ((SPI_FLASH_SIZE_MAP == 0) || (SPI_FLASH_SIZE_MAP == 1))
#error "The flash map is not supported"
#elif (SPI_FLASH_SIZE_MAP == 2)
#define SYSTEM_PARTITION_OTA_SIZE			0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR			0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR			0xfb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR			0xfc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR		0xfd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR       0x7c000
#elif (SPI_FLASH_SIZE_MAP == 3)
#define SYSTEM_PARTITION_OTA_SIZE			0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR			0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR			0x1fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR			0x1fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR		0x1fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR       0x7c000
#elif (SPI_FLASH_SIZE_MAP == 4)
#define SYSTEM_PARTITION_OTA_SIZE			0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR			0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR			0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR			0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR		0x3fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR       0x7c000
#elif (SPI_FLASH_SIZE_MAP == 5)
#define SYSTEM_PARTITION_OTA_SIZE			0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR			0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR			0x1fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR			0x1fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR		0x1fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR       0xfc000
#elif (SPI_FLASH_SIZE_MAP == 6)
#define SYSTEM_PARTITION_OTA_SIZE			0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR			0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR			0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR			0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR		0x3fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR       0xfc000
#else
#error "The flash map is not supported"
#endif

#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM            SYSTEM_PARTITION_CUSTOMER_BEGIN

uint32 priv_param_start_sec;
static const partition_item_t at_partition_table[] = {
	{ SYSTEM_PARTITION_BOOTLOADER, 		0x0, 												0x1000},
	{ SYSTEM_PARTITION_OTA_1,   		0x1000, 											SYSTEM_PARTITION_OTA_SIZE},
	{ SYSTEM_PARTITION_OTA_2,   		SYSTEM_PARTITION_OTA_2_ADDR, SYSTEM_PARTITION_OTA_SIZE},
	{ SYSTEM_PARTITION_RF_CAL,  		SYSTEM_PARTITION_RF_CAL_ADDR, 		0x1000},
	{ SYSTEM_PARTITION_PHY_DATA, 		SYSTEM_PARTITION_PHY_DATA_ADDR, 	0x1000},
	{ SYSTEM_PARTITION_SYSTEM_PARAMETER, 	SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR, 0x3000},
	{ SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM, SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR, 0x1000},
};
os_timer_t relay;
os_timer_t esp_config;
os_timer_t http_update;
int relay_arm_interval = RELAY_ARM_INTERVAL;
int tz = TIMEZONE;
uint8 secondsOn[60], minutesOn[60], hoursOn[24], daysOn[31], monthsOn[12], dweekOn[7];
uint8 secondsOff[60], minutesOff[60], hoursOff[24], daysOff[31], monthsOff[12], dweekOff[7];

void relay_cb(void *);
void esp_config_cb(void *);
void http_update_cb(void *);
void connect_cb(void *);
void reconnect_cb(void *, sint8);
void disconnect_cb(void *);
uint8 parse_body(char *);
void receive_cb(void *, char *, unsigned short);
void sent_cb(void *);
void web_server_found(const char *, ip_addr_t *, void *);
void wifi_event_cb(System_Event_t *);
void set_enhanced_boot_mode();
void wifi_scan_cb(void *, STATUS);
void sysinfo();
void configure_timers();
uint8 cron_parser(char *, bool);

void ICACHE_FLASH_ATTR user_rf_pre_init(void) {
	/*rf calibration after power up:
	- 0 - see byte 115 in esp_init_data_default.bin, which is 1 in _v08.bin
	(obtained using hexdump -e '"%_ad "' -b -s 115 esp_init_data_default_v08.bin)
	- 1 - only calibrate VDD33 and Tx power which will take about 18ms
	- 2 - only calibrate VDD33 (3.3V power supply) which will take about 2ms
	Option 2 may not be an option unless TOUT pin is suspended and byte 108 (vdd33_const)
	is set to 0xFF
	- 3 - will do the whole RF calibration which will take about 200ms
	*/
	system_phy_set_powerup_option(3);
	/*
	Set maximum value of RF TX power in 0.25 dBm units in the range 0 - 82
	Byte 35 - target_power_qdb_0 - in esp_init_data_default.bin
	In _v08.bin it is set to 74 (18dBm or 63mW)
	*/
	system_phy_set_max_tpw(WIFI_POWER);
	/*
	Alternatively power can be adjusted in VDD33 units (1/1024 V)
	in the range 1900 - 3300
	To get the current power supply voltage, use system_get_vdd33()
	For it to work TOUT pin must be suspended and 108 byte (vdd33_const) in
	esp_init_data_default.bin set to 0xFF. In _v08.bin it is set to 0, meaning
	that calibration will not be done and defaults to 3.3V.
	esp8266 can work with a range of voltages from 1.8V to 3.6V, so the need for
	power calibration
	*/
	//uint16 voltage = system_get_vdd33(); //in 1/1024V units
	//system_phy_set_tpw_via_vdd33(uint16 vdd33);
}
uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void) {
	enum flash_size_map size_map = system_get_flash_size_map();
	uint32 rf_cal_sec = 0;
	switch (size_map) {
	case FLASH_SIZE_4M_MAP_256_256:
		rf_cal_sec = 128 - 5;
		priv_param_start_sec = 0x3C;
		break;
	case FLASH_SIZE_8M_MAP_512_512:
		rf_cal_sec = 256 - 5;
		priv_param_start_sec = 0x7C;
		break;
	case FLASH_SIZE_16M_MAP_512_512:
		rf_cal_sec = 512 - 5;
		priv_param_start_sec = 0x7C;
		break;
	case FLASH_SIZE_16M_MAP_1024_1024:
		rf_cal_sec = 512 - 5;
		priv_param_start_sec = 0xFC;
		break;
	case FLASH_SIZE_32M_MAP_512_512:
		rf_cal_sec = 1024 - 5;
		priv_param_start_sec = 0x7C;
		break;
	case FLASH_SIZE_32M_MAP_1024_1024:
		rf_cal_sec = 1024 - 5;
		priv_param_start_sec = 0xFC;
		break;
	case FLASH_SIZE_64M_MAP_1024_1024:
		rf_cal_sec = 2048 - 5;
		priv_param_start_sec = 0xFC;
		break;
	case FLASH_SIZE_128M_MAP_1024_1024:
		rf_cal_sec = 4096 - 5;
		priv_param_start_sec = 0xFC;
		break;
	default:
		rf_cal_sec = 0;
		priv_param_start_sec = 0;
		break;
	}
	return rf_cal_sec;
}
void ICACHE_FLASH_ATTR user_pre_init(void) {
	if (!system_partition_table_regist(at_partition_table, sizeof(at_partition_table) / sizeof(at_partition_table[0]), SPI_FLASH_SIZE_MAP)) {
		os_printf("system_partition_table_regist failed\n");
		while (1);
	}
}

void ICACHE_FLASH_ATTR user_init(void) {
	if (system_get_boot_mode() != 0) set_enhanced_boot_mode();
	if (wifi_set_opmode_current(1)) //station
		os_printf("wifi set station mode ok\n");
	else os_printf("wifi set station mode failed\n");
	enum phy_mode wifi_mode = PHY_MODE_11G;
	if (wifi_set_phy_mode(wifi_mode)) {
		switch (wifi_mode) {
		case PHY_MODE_11B:
			os_printf("wifi set 802.11b (<11Mbps, 35-140m) mode ok\n");
			break;
		case PHY_MODE_11G:
			os_printf("wifi set 802.11g (<54Mbps, 38-140m) mode ok\n");
			break;
		case PHY_MODE_11N:
			os_printf("wifi set 802.11n (<289/600Mbps, 70-250m) mode ok\n");
			break;
		default:
			os_printf("wifi mode is unknown\n");
		}
	} else os_printf("wifi set phy_mode %d failed (1: b, 2: g, 3: n)\n", wifi_mode);
	char ssid[32] = SSID;
	char password[64] = PASSPHRASE;
	struct station_config stationConf;
	stationConf.bssid_set = 0; //ignore AP MAC
	os_memcpy(&stationConf.ssid, ssid, 32);
	os_memcpy(&stationConf.password, password, 64);
	
	if (wifi_station_set_config_current(&stationConf))
		os_printf("configure wifi ok\n");
	else os_printf("configure wifi failed\n");

	wifi_set_event_handler_cb(wifi_event_cb);
}

void ICACHE_FLASH_ATTR esp_config_cb(void *arg) {
	os_timer_disarm(&esp_config);
	if (wifi_station_get_connect_status() == STATION_GOT_IP) {
		sysinfo();
		configure_timers();
		os_printf("station got IP\n");
		if (wifi_station_set_hostname(HOSTNAME))
			os_printf("set hostname %s ok\n", HOSTNAME);
		else os_printf("set hostname %s HOSTNAME failed\n", HOSTNAME);

		if (USE_STATIC_IP) {
			if (wifi_station_dhcpc_stop())
				os_printf("stop dhcp client ok\n");
			else os_printf("stop dhcp client failed\n");

			struct ip_info ip;
			ipaddr_aton(STATIC_IP, &ip.ip);
			ipaddr_aton(GATEWAY_IP, &ip.gw);
			ipaddr_aton(NETMASK, &ip.netmask);

			if (wifi_set_ip_info(STATION_IF, &ip))
				os_printf("set static ip %s ok\n", ipaddr_ntoa(&ip.ip));
			else os_printf("set static ip %s failed\n", ipaddr_ntoa(&ip.ip));
		}

		ip_addr_t *dns_ip = NULL;
		dns_ip = (ip_addr_t *)os_zalloc(sizeof(ip_addr_t));
		ipaddr_aton(DNS, dns_ip);
		espconn_dns_setserver(0, dns_ip);
		os_free(dns_ip); dns_ip = NULL;
		ip_addr_t dns_addr = espconn_dns_getserver(0);
		os_printf("dns 0: %s\n", ipaddr_ntoa(&dns_addr));

		sntp_init();

		sntp_setservername(0, NTP0);
		sntp_setservername(1, NTP1);
		sntp_setservername(2, NTP2);

		if (sntp_set_timezone(tz))
			os_printf("set timezone %d ok\n", tz);
		else os_printf("set timezone %d failed\n", tz);

		os_printf("sntp 0: %s\n", sntp_getservername(0));
		os_printf("sntp 1: %s\n", sntp_getservername(1));
		os_printf("sntp 2: %s\n", sntp_getservername(2));

		char par[os_strlen(TIMER_ON_CRONLINE) + os_strlen(TIMER_OFF_CRONLINE) + 15];
		os_sprintf(par, "%s\n%s\n%u\n%d\n", TIMER_ON_CRONLINE, TIMER_OFF_CRONLINE, RELAY_ARM_INTERVAL, TIMEZONE);
		parse_body(par);

		if (HTTP_UPDATE) http_update_cb(NULL);
		else os_timer_arm(&relay, relay_arm_interval, 0);
	}
	else os_timer_arm(&esp_config, ESP_CONFIG_ARM_INTERVAL, 0);
}
void ICACHE_FLASH_ATTR relay_cb(void *arg) {
	uint32 current_stamp = 0;
	char * datetime = NULL;
	char * datetime_field[4];//do not need year
	char * hms[3];
	bool on = true, off = true;
	uint8 i, j;

	os_timer_disarm(&relay);
	current_stamp = sntp_get_current_timestamp();
	if (current_stamp > 0) {
		datetime = sntp_get_real_time(current_stamp);
		os_printf("datetime %s\n", datetime);
		//datetime Sat May 16 05:17:04 2020
		i = 0;
		datetime_field[i] = strtok(datetime, " ");
		while (datetime_field[i] && i < 3) {
			i++;
			datetime_field[i] = strtok(NULL, " ");
		}
		i = 0;
		if (!datetime_field[3]) {
			os_printf("turn_relay_on: hms is NULL - datetime is invalid\n");
			return;
		}
		hms[i] = strtok(datetime_field[3], ":");
		while (hms[i] && i < 2) {
			i++;
			hms[i] = strtok(NULL, ":");
		}
		for (i = 0; i <= 3; i++) {
			switch (i) {
			case 0: //day-of-the-week
				if (!datetime_field[i]) {
					os_printf("turn_relay_on: day-of-week is NULL\n");
					return;
				}
				if (os_strncmp(datetime_field[i], "Sun", 3) == 0) j = 0;
				else if (os_strncmp(datetime_field[i], "Mon", 3) == 0) j = 1;
				else if (os_strncmp(datetime_field[i], "Tue", 3) == 0) j = 2;
				else if (os_strncmp(datetime_field[i], "Wed", 3) == 0) j = 3;
				else if (os_strncmp(datetime_field[i], "Thu", 3) == 0) j = 4;
				else if (os_strncmp(datetime_field[i], "Fri", 3) == 0) j = 5;
				else if (os_strncmp(datetime_field[i], "Sat", 3) == 0) j = 6;
				else {
					os_printf("turn_relay_on: invalid day-of-week\n");
					return;
				}
				if (!dweekOn[j]) on = false;
				if (!dweekOff[j]) off = false;
				break;
			case 1: //Month
				if (!datetime_field[i]) {
					os_printf("turn_relay_on: month is NULL\n");
					return;
				}
				if (os_strncmp(datetime_field[i], "Jan", 3) == 0) j = 0;
				else if (os_strncmp(datetime_field[i], "Feb", 3) == 0) j = 1;
				else if (os_strncmp(datetime_field[i], "Mar", 3) == 0) j = 2;
				else if (os_strncmp(datetime_field[i], "Apr", 3) == 0) j = 3;
				else if (os_strncmp(datetime_field[i], "May", 3) == 0) j = 4;
				else if (os_strncmp(datetime_field[i], "Jun", 3) == 0) j = 5;
				else if (os_strncmp(datetime_field[i], "Jul", 3) == 0) j = 6;
				else if (os_strncmp(datetime_field[i], "Aug", 3) == 0) j = 7;
				else if (os_strncmp(datetime_field[i], "Sep", 3) == 0) j = 8;
				else if (os_strncmp(datetime_field[i], "Oct", 3) == 0) j = 9;
				else if (os_strncmp(datetime_field[i], "Nov", 3) == 0) j = 10;
				else if (os_strncmp(datetime_field[i], "Dec", 3) == 0) j = 11;
				else {
					os_printf("turn_relay_on: invalid month\n");
					return;
				}
				if (!monthsOn[j]) on = false;
				if (!monthsOff[j]) off = false;
				break;
			case 2: //day
				if (!datetime_field[i]) {
					os_printf("turn_relay_on: day is NULL\n");
					return;
				}
				if (isdigit(datetime_field[i][0]))
					j = atoi(datetime_field[i]) - 1;
				else {
					os_printf("turn_relay_on: days is not a number\n");
					return;
				}
				if (j > 30) {
					os_printf("turn_relay_on: days are from 1 to 31\n"); 
					return;
				}
				if (!daysOn[j])	on = false;
				if (!daysOff[j]) off = false;
				break;
			case 3: //hms
				//hour
				if (!hms[0]) {
					os_printf("turn_relay_on: hours is NULL\n");
					return;
				}
				if (isdigit(hms[0][0]))
					j = atoi(hms[0]);
				else {
					os_printf("turn_relay_on: hours is not a number\n"); return;
				}
				if (j > 23) {
					os_printf("turn_relay_on: hours are from 0 to 23\n"); return;
				}
				if (!hoursOn[j]) on = false;
				if (!hoursOff[j]) off = false;

				//minute
				if (!hms[1]) {
					os_printf("turn_relay_on: minutes is NULL\n");
					return;
				}
				if (isdigit(hms[1][0]))
					j = atoi(hms[1]);
				else {
					os_printf("turn_relay_on: minutes is not a number\n"); 
					return;
				}
				if (j > 59) {
					os_printf("turn_relay_on: minutes are from 0 to 59\n"); 
					return;
				}
				if (!minutesOn[j]) on = false;
				if (!minutesOff[j]) off = false;

				//seconds
				if (!hms[2]) {
					os_printf("turn_relay_on: seconds is NULL\n");
					return;
				}
				if (isdigit(hms[2][0]))
					j = atoi(hms[2]);
				else {
					os_printf("turn_relay_on: seconds is not a number\n"); 
					return;
				}
				if (j > 59) {
					os_printf("turn_relay_on: seconds are from 0 to 59\n"); 
					return;
				}
				if (!secondsOn[j]) on = false;
				if (!secondsOff[j]) off = false;
				break;
			}
			if (!on && !off) break;
		}
		if (on) {
			os_printf("turning relay on...\n");
			gpio_output_set(0, BIT0, BIT0, 0); //Set GPIO0 as low - level output.
		}
		else if (off) {
			os_printf("turning relay off...\n");
			gpio_output_set(BIT0, 0, BIT0, 0); //Set GPIO0 as high - level output.
		}
	}
	else {
		os_printf("unable to get time from sntp server\n"); 
		//sntp_init();
	}
	os_timer_arm(&relay, relay_arm_interval, 0);
}
void ICACHE_FLASH_ATTR http_update_cb(void * arg) {
	os_timer_disarm(&http_update);
	os_timer_arm(&http_update, HTTP_UPDATE_INTERVAL, 0);
	os_printf("set timer parameters update over http interval to %ums\n", HTTP_UPDATE_INTERVAL);
	struct espconn * con = (struct espconn *)os_zalloc(sizeof(struct espconn));
	con->type = ESPCONN_TCP;
	con->state = ESPCONN_NONE;
	con->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
	con->proto.tcp->local_port = espconn_port();
	con->proto.tcp->remote_port = atoi(WEB_SERVER_PORT);
	ipaddr_aton(STATIC_IP, con->proto.tcp->local_ip);

	ip_addr_t web_server_ip;
	err_t err = espconn_gethostbyname(con, WEB_SERVER, &web_server_ip, web_server_found);
	switch (err) {
	case ESPCONN_OK:
		//got a response or the IP is already in cache
		os_printf("gethostbyname(%s): ok\n", WEB_SERVER);
		break;
	case ESPCONN_INPROGRESS:
		os_printf("gethostbyname(%s): waiting for DNS server response\n", WEB_SERVER);
		break;
	case ESPCONN_ARG:
		os_printf("gethostbyname(%s) error: timeout, invalid arg or dns client is not set up\n", WEB_SERVER);
		break;
	default:
		os_printf("gethostbyname(%s) error: %d\n", WEB_SERVER, err);
		break;
	}
}
void ICACHE_FLASH_ATTR connect_cb(void * arg) {
	struct espconn * con = (struct espconn *)arg;
	if (!con) return;
	espconn_regist_recvcb(con, receive_cb);
	espconn_regist_sentcb(con, sent_cb);
	char sdk_version[32];
	os_bzero(sdk_version, sizeof(sdk_version));
	os_sprintf(sdk_version, "%s", system_get_sdk_version());
	char buf[68 + strlen(HTTP_METHOD) + strlen(WEB_SERVER_PATH) + strlen(HTTP_VERSION) + strlen(WEB_SERVER) + strlen(WEB_SERVER_PORT) + strlen(USER_AGENT) + strlen(sdk_version)];
	uint16 l = os_sprintf(buf, "%s %s %s\r\nHost: %s:%s\r\nConnection: close\r\nUser-Agent: %s/%s\r\nAccept: text/plain\r\n\r\n", HTTP_METHOD, WEB_SERVER_PATH, HTTP_VERSION, WEB_SERVER, WEB_SERVER_PORT, USER_AGENT, sdk_version);
	os_printf("sending GET request:\n%s", buf);
	sint8 stat = espconn_send(con, (uint8 *)buf, l);
	switch (stat) {
	case 0: 
		os_printf("GET request succeeded\n");
		break;
	case ESPCONN_MEM:
		os_printf("GET request error: out of memory\n");
		break;
	case ESPCONN_ARG:
		os_printf("GET request error: illegal argument\n");
		break;
	case ESPCONN_MAXNUM:
		os_printf("GET request error: buffer full\n");
		break;
	case ESPCONN_IF:
		os_printf("GET request error: failed to send UDP data\n");
		break;
	}
}
void ICACHE_FLASH_ATTR reconnect_cb(void * args, sint8 err) {
	os_printf("reconnect_cb called\n");
}
void ICACHE_FLASH_ATTR disconnect_cb(void * args) {
	struct espconn *con = (struct espconn *)args;
	if (con) {
		os_printf("disconnecting from %d.%d.%d.%d\n", con->proto.tcp->remote_ip[0], con->proto.tcp->remote_ip[1], con->proto.tcp->remote_ip[2], con->proto.tcp->remote_ip[3]);
		espconn_delete(con);
		if (con->proto.tcp != NULL) {
			os_free(con->proto.tcp);
		}
		os_free(con);
	}
}
void ICACHE_FLASH_ATTR sent_cb(void * args) {
	struct espconn * con = (struct espconn *)args;
	if (con && con->type == ESPCONN_TCP) {
		os_printf("sent request to %d.%d.%d.%d:%d\n", con->proto.tcp->remote_ip[0], con->proto.tcp->remote_ip[1], con->proto.tcp->remote_ip[2], con->proto.tcp->remote_ip[3], con->proto.tcp->remote_port);
		os_printf("connection state: ");
		switch (con->state) {
		case ESPCONN_NONE:
			os_printf("not connected\n");
			break;
		case ESPCONN_WAIT:
			os_printf("waiting\n");
			break;
		case ESPCONN_LISTEN:
			os_printf("listening\n");
			break;
		case ESPCONN_CONNECT:
			os_printf("connected\n");
			break;
		case ESPCONN_WRITE:
			os_printf("sending\n");
			break;
		case ESPCONN_READ:
			os_printf("receiving\n");
			break;
		case ESPCONN_CLOSE:
			os_printf("closed\n");
			break;
		default:
			os_printf("unknown state\n");
		}
	}
}
uint8 ICACHE_FLASH_ATTR parse_body(char * body) {
	char * cronlineOn = NULL;
	char * cronlineOff = NULL;
	char * relay_arm_interval_char = NULL;
	char * timezone = NULL;
	uint8 e;
	os_memset(secondsOn, 0, 60);
	os_memset(minutesOn, 0, 60);
	os_memset(hoursOn, 0, 24);
	os_memset(daysOn, 0, 31);
	os_memset(monthsOn, 0, 12);
	os_memset(secondsOff, 0, 60);
	os_memset(minutesOff, 0, 60);
	os_memset(hoursOff, 0, 24);
	os_memset(daysOff, 0, 31);
	os_memset(monthsOff, 0, 12);
//	os_printf("body:\n%s\n", body);

	cronlineOn = strtok(body, "\r\n");
	if (!cronlineOn) return 1;
//	os_printf("cronlineOn %s\n", cronlineOn);
	cronlineOff = strtok(NULL, "\r\n");
	if (!cronlineOff) return 2;
//	os_printf("cronlineOff %s\n", cronlineOff);
	relay_arm_interval_char = strtok(NULL, "\r\n");
	if (!relay_arm_interval_char) return 3;
//	os_printf("relay_arm_interval_char %s\n", relay_arm_interval_char);
	if (isdigit(relay_arm_interval_char[0])) 
		relay_arm_interval = atoi(relay_arm_interval_char);
	else return 4;
	timezone = strtok(NULL, "\r\n");
	if (!timezone) return 5;
//	os_printf("timezone %s\n", timezone);
	tz = atoi(timezone);

	e = cron_parser(cronlineOn, true);
	if (e != 0) os_printf("cron_parser(cronlineOn) error %u\n", e);
	e = cron_parser(cronlineOff, false);
	if (e != 0) os_printf("cron_parser(cronlineOff) error %u\n", e);
}
void ICACHE_FLASH_ATTR receive_cb(void * args, char * buf, unsigned short len) {
	struct espconn *con = (struct espconn *)args;

	if (!buf) {
		os_printf("receive_cb error: buffer is NULL\n");
		return;
	}
	int http_status = -1;
	char * body;
	char * cl = NULL;
	int content_len = 0;
	uint8 e;
	if (buf[0] != '\0') {
		cl = os_strstr(buf, "Content-Length: ");
		if (cl) content_len = atoi(cl + 16);
		if (os_strncmp(buf, HTTP_VERSION, os_strlen(HTTP_VERSION)) == 0) {
			http_status = atoi(buf + os_strlen(HTTP_VERSION) + 1);
			if (http_status == 200) {
				body = os_strstr(buf, "\r\n\r\n") + 4;
				int body_len = os_strlen(body);
				if (body_len == content_len)
					parse_body(body);
			} else os_printf("receive_cb error: http_status %d\n", http_status);
		}
		else {
			e = parse_body(buf);
			if (e != 0) os_printf("parse_body error %u\n", e);
		}
		os_free(buf);
	} else os_printf("receive_cb: http response is empty\n");
	if (tz != TIMEZONE) {
		if (sntp_set_timezone(tz))
			os_printf("sntp set timezone %d ok\n", tz);
		else os_printf("sntp set timezone %d failed\n", tz);
	}
	os_timer_arm(&relay, relay_arm_interval, 0);
}
void ICACHE_FLASH_ATTR web_server_found(const char * name, ip_addr_t * ip, void * args) {
	if (ip) {
		struct espconn *con = (struct espconn *)args;
		os_memcpy(con->proto.tcp->remote_ip, ip, 4);
		os_printf("web server %s ip is " IPSTR "\n", name, IP2STR(con->proto.tcp->remote_ip));

		if (espconn_regist_connectcb(con, connect_cb) == 0)
			os_printf("registered connect_cb()\n");
		else os_printf("failed to register connect_cb()\n");
		if (espconn_regist_reconcb(con, reconnect_cb) == 0)
			os_printf("registered reconnect_cb()\n");
		else os_printf("failed to register reconnect_cb()\n");
		if (espconn_regist_disconcb(con, disconnect_cb) == 0)
			os_printf("registered disconnect_cb\n");
		else os_printf("failed to register disconnect_cb\n");
		sint8 res = espconn_connect(con);
		switch (res) {
		case 0:
			os_printf("connecting to the web server...\n");
			break;
		case ESPCONN_RTE:
			os_printf("espconn_connect error: Routing Problem\n");
			break;
		case ESPCONN_MEM:
			os_printf("espconn_connect error: Out of memory\n");
			break;
		case ESPCONN_ISCONN:
			os_printf("espconn_connect error: Already connected\n");
			break;
		case ESPCONN_ARG:
			os_printf("espconn_connect error: Illegal argument\n");
			break;
		}
	}
}
void ICACHE_FLASH_ATTR wifi_event_cb(System_Event_t *evt)
{
	//os_printf("event %x\n", evt->event);
	char * reason;
	switch (evt->event) {
	case EVENT_STAMODE_CONNECTED:
		os_printf("connected to ssid %s, channel %d\n",
			evt->event_info.connected.ssid,
			evt->event_info.connected.channel);
		os_timer_disarm(&esp_config);
		os_timer_setfn(&esp_config, (os_timer_func_t *)esp_config_cb, NULL);
		os_timer_arm(&esp_config, ESP_CONFIG_ARM_INTERVAL, 0);
		break;
	case EVENT_STAMODE_DISCONNECTED:
		switch (evt->event_info.disconnected.reason) {
		case REASON_UNSPECIFIED: reason = "unknown"; break;
		case REASON_AUTH_EXPIRE: reason = "auth expired"; break;
		case REASON_AUTH_LEAVE: reason = "left authentication"; break;
		case REASON_ASSOC_EXPIRE: reason = "assoc expired"; break;
		case REASON_ASSOC_TOOMANY: reason = "too many assoc"; break;
		case REASON_NOT_AUTHED: reason = "not authenticated"; break;
		case REASON_NOT_ASSOCED: reason = "not associated"; break;
		case REASON_ASSOC_LEAVE: reason = "left association"; break;
		case REASON_ASSOC_NOT_AUTHED: reason = "association not authenticated"; break;
		case REASON_DISASSOC_PWRCAP_BAD: reason = "disassociated - bad pwr cap"; break;
		case REASON_DISASSOC_SUPCHAN_BAD: reason = "disassociated - bad sup channel"; break;
		case REASON_IE_INVALID: reason = "invalid IE"; break;
		case REASON_MIC_FAILURE: reason = "misc failure"; break;
		case REASON_4WAY_HANDSHAKE_TIMEOUT: reason = "4-way handshake timeout"; break;
		case REASON_GROUP_KEY_UPDATE_TIMEOUT: reason = "group key update timeout"; break;
		case REASON_IE_IN_4WAY_DIFFERS: reason = "IE in 4-way differs"; break;
		case REASON_GROUP_CIPHER_INVALID: reason = "invalid group cipher"; break;
		case REASON_PAIRWISE_CIPHER_INVALID: reason = "invalid pairwise cipher"; break;
		case REASON_AKMP_INVALID: reason = "invalid AKMP"; break;
		case REASON_UNSUPP_RSN_IE_VERSION: reason = "unsupported RSN IE version"; break;
		case REASON_INVALID_RSN_IE_CAP: reason = "invalid RSN IE cap"; break;
		case REASON_802_1X_AUTH_FAILED: reason = "802.1x auth failed"; break;
		case REASON_CIPHER_SUITE_REJECTED: reason = "cipher suite rejected"; break;
		case REASON_BEACON_TIMEOUT: reason = "beacon timeout"; break;
		case REASON_NO_AP_FOUND: reason = "no AP found"; break;
		case REASON_AUTH_FAIL: reason = "authentication failed"; break;
		case REASON_ASSOC_FAIL: reason = "association failed"; break;
		case REASON_HANDSHAKE_TIMEOUT: reason = "handshake timeout"; break;
		}
		os_printf("disconnected from ssid %s, reason %s, re-scanning...\n",
			evt->event_info.disconnected.ssid,
			reason);
		struct scan_config config;
		config.ssid = SSID;
		config.bssid = NULL;
		config.channel = SSID_CHANNEL;
		config.show_hidden = 1;
		config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
		config.scan_time.active.min = MIN_SCANTIME;
		config.scan_time.active.max = MAX_SCANTIME;
		wifi_station_scan(&config, wifi_scan_cb);
		break;
	case EVENT_STAMODE_AUTHMODE_CHANGE:
		os_printf("mode: %d -> %d\n",
			evt->event_info.auth_change.old_mode,
			evt->event_info.auth_change.new_mode);
		break;
	case EVENT_STAMODE_GOT_IP:
		os_printf("got ip:" IPSTR ", mask:" IPSTR ", gw:" IPSTR "\n",
			IP2STR(&evt->event_info.got_ip.ip),
			IP2STR(&evt->event_info.got_ip.mask),
			IP2STR(&evt->event_info.got_ip.gw));
		break;
	case EVENT_OPMODE_CHANGED:
		os_printf("wifi mode changed from %u to %u\n", &evt->event_info.opmode_changed.old_opmode, &evt->event_info.opmode_changed.new_opmode);
		break;
	default:
		break;
	}
}
void set_enhanced_boot_mode(void) {
	uint8 boot_version = system_get_boot_version();
	os_printf("boot version %u\n", boot_version);

	//if boot version >=3, it is possible to enable enhanced boot mode
	//by calling system_restart_enhance(0, 0x01000 or 0x81000)
	if (boot_version >= 3) {
		if (system_restart_enhance(SYS_BOOT_NORMAL_BIN, USER1_BIN_ADDRESS))
			os_printf("system_restart_enhance() ok\n");
		else os_printf("system_restart_enhance() failed\n");
	}
}
void ICACHE_FLASH_ATTR wifi_scan_cb(void * args, STATUS status) {
	os_printf("wifi_scan_cb() called\n");
	struct bss_info * bss = NULL, * bss_prev = NULL;

	char ssid[33], * auth_mode, * pairwise_cipher, * group_cipher;
	if (status == OK) {
		bss = (struct bss_info *)args;
		do {
			os_printf("bssid %x:%x:%x:%x:%x:%x ", bss->bssid[0], bss->bssid[1], bss->bssid[2], bss->bssid[3], bss->bssid[4], bss->bssid[5]);			
			os_memset(ssid, 0, 33);
			os_strncpy(ssid, bss->ssid, bss->ssid_len);
			switch (bss->authmode) {
			case AUTH_OPEN: auth_mode = "open"; break;
			case AUTH_WEP: auth_mode = "WEP"; break;
			case AUTH_WPA_PSK: auth_mode = "WPA_PSK"; break;
			case AUTH_WPA2_PSK: auth_mode = "WPA2_PSK"; break;
			case AUTH_WPA_WPA2_PSK: auth_mode = "WPA_WPA2_PSK"; break;
			case AUTH_MAX: auth_mode = "MAX"; break;
			}
			switch (bss->pairwise_cipher) {
			case CIPHER_NONE: pairwise_cipher = "none"; break;
			case CIPHER_WEP40: pairwise_cipher = "WEP40"; break;
			case CIPHER_WEP104: pairwise_cipher = "WEP104"; break;
			case CIPHER_TKIP: pairwise_cipher = "TKIP"; break;
			case CIPHER_CCMP: pairwise_cipher = "CCMP"; break;
			case CIPHER_TKIP_CCMP: pairwise_cipher = "TKIP_CCMP"; break;
			case CIPHER_UNKNOWN: pairwise_cipher = "unknown"; break;
			}
			switch (bss->group_cipher) {
			case CIPHER_NONE: group_cipher = "none"; break;
			case CIPHER_WEP40: group_cipher = "WEP40"; break;
			case CIPHER_WEP104: group_cipher = "WEP104"; break;
			case CIPHER_TKIP: group_cipher = "TKIP"; break;
			case CIPHER_CCMP: group_cipher = "CCMP"; break;
			case CIPHER_TKIP_CCMP: group_cipher = "TKIP_CCMP"; break;
			case CIPHER_UNKNOWN: group_cipher = "unknown"; break;
			}
			os_printf("ssid %s, channel %u, hidden %u, freq_offset %d, freqcal_val %d, auth mode %s, pairwise cipher %s, group cipher %s\n", ssid, bss->channel, bss->is_hidden, bss->freq_offset, bss->freqcal_val, auth_mode, pairwise_cipher, group_cipher);
			bss_prev = bss;
			bss = (struct bss_info *)&(bss->next);
		} while (bss != bss_prev);
		if (wifi_station_connect()) os_printf("Re-connected\n");
		else os_printf("wifi_station_connect() failed\n");
	}
}
void ICACHE_FLASH_ATTR sysinfo(void) {
	os_printf("cpu frequency %u\n", system_get_cpu_freq());
	//Overclock CPU (default is SYS_CPU_80MHz)
	//system_update_cpu_freq(SYS_CPU_160MHz);

	system_print_meminfo();
	os_printf("free heap size %u\n", system_get_free_heap_size());
}
void ICACHE_FLASH_ATTR configure_timers(void) {
	os_timer_disarm(&relay);
	os_timer_disarm(&http_update);
	os_timer_setfn(&relay, (os_timer_func_t *)relay_cb, NULL);
	os_timer_setfn(&http_update, (os_timer_func_t *)http_update_cb, NULL);
}

uint8 ICACHE_FLASH_ATTR cron_parser(char * cronline, bool onTimer) {
	//cronline <seconds> <minutes> <hours> <days> <months> <days-of-the-week>
	char * cron_field[6];
	char * cron_subfield[6][60];
	char * subfield_min = NULL;
	char * subfield_max = NULL;
	uint8 i, j, k, min, max;
	bool range;
	i = 0;
	cron_field[i] = strtok(cronline, " ");
	while (cron_field[i] && i < 5) {
		i++;
		cron_field[i] = strtok(NULL, " ");
	}
	for (i = 0; i < 6; i++) {
		j = 0;
		if (!cron_field[i]) break;
		//os_printf("cron_field[%u] %s\n", i, cron_field[i]);
		cron_subfield[i][j] = strtok(cron_field[i], ",");
		while (cron_subfield[i][j] && j < 59) {
			j++;
			cron_subfield[i][j] = strtok(NULL, ",");
		}
	}
	for (i = 0; i < 6; i++) {
		for (j = 0; j < 60; j++) {
			if (!cron_subfield[i][j]) break;
			//os_printf("cron_subfield[%u][%u] %s\n", i, j, cron_subfield[i][j]);
			min = 255;
			subfield_min = strtok(cron_subfield[i][j], "-");
			if (!subfield_min) break;
			//os_printf("subfield_min %s\n", subfield_min);
			if (os_strncmp(subfield_min, "*", 1) != 0) {
				if (!isdigit(subfield_min[0])) return 1; 
				min = atoi(subfield_min);
				subfield_max = strtok(NULL, "-");
				if (subfield_max) {
					//os_printf("subfield_max %s\n", subfield_max);
					if (!isdigit(subfield_max[0])) return 2;
					max = atoi(subfield_max);
				} else max = min;
			}
			switch (i) {
				case 0: //seconds
					if (min == 255) { min = 0; max = 59;	}
					if (min >= 60 || max >= 60) return 3;
					for (k = min; k <= max; k++) {
						if (onTimer) secondsOn[k] = 1;
						else secondsOff[k] = 1;
					}
					break;
				case 1: //minutes
					if (min == 255) { min = 0; max = 59; }
					if (min >= 60 || max >= 60) return 4;
					for (k = min; k <= max; k++) {
						if (onTimer) minutesOn[k] = 1;
						else minutesOff[k] = 1;
					}
					break;
				case 2: //hours
					if (min == 255) { min = 0; max = 23; }
					if (min >= 24  || max >= 24) return 5;
					for (k = min; k <= max; k++) {
						if (onTimer) hoursOn[k] = 1;
						else hoursOff[k] = 1;
					}
					break;
				case 3: //days
					if (min == 255) { min = 1; max = 31; }
					min--; max--;
					if (min >= 31 || max >= 31) return 6;
					for (k = min; k <= max; k++) {
						if (onTimer) daysOn[k] = 1;
						else daysOff[k] = 1;
					}
					break;
				case 4: //months
					if (min == 255) { min = 1; max = 12; }
					min--; max--;
					if (min >= 12 || max >= 12) return 7;
					for (k = min; k <= max; k++) {
						if (onTimer) monthsOn[k] = 1;
						else monthsOff[k] = 1;
					}
					break;
				case 5: //day-of-week
					if (min == 255) { min = 0; max = 6; }
					if (min >= 7 || max >= 7) return 8;
					for (k = min; k <= max; k++) {
						if (onTimer) dweekOn[k] = 1;
						else dweekOff[k] = 1;
					}
					break;
			}
		}
	}
	return 0;
}
