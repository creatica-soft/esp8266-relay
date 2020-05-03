/*
Review and update defines
mkdir -p ESP8266_NONOS_SDK-2.2.1/timer/user
cp user_main.c ESP8266_NONOS_SDK-2.2.1/timer/user
cp ESP8266_NONOS_SDK-2.2.1/examples/IoT_Demo/Makefile ESP8266_NONOS_SDK-2.2.1/timer
cp -r ESP8266_NONOS_SDK-2.2.1/examples/IoT_Demo/include ESP8266_NONOS_SDK-2.2.1/timer/
cp SP8266_NONOS_SDK-2.2.1/examples/IoT_Demo/user/Makefile ESP8266_NONOS_SDK-2.2.1/timer/user 
Install esp-open-sdk first for crosscompiling!
export PATH=/home/apoliakevitch/esp-open-sdk/xtensa-lx106-elf/bin:$PATH
Compile user1.bin for esp-01s with "make COMPILE=gcc BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2"
make clean
Compile user2.bin for esp-01s with "make clean; make COMPILE=gcc BOOT=new APP=2 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2"
Ground GPIO0 before flashing
Install esptool for flashing ESP
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x01000 ../ESP8266_NONOS_SDK-2.2.1/bin/upgrade/user1.1024.new.2.bin
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x81000 ../ESP8266_NONOS_SDK-2.2.1/bin/upgrade/user2.1024.new.2.bin
Unground GPIO0 for normal operation
miniterm --raw /dev/ttyAMA0 74880 - to see the os_printf() messages
*/
#include "ets_sys.h"
#include "sntp.h"
#include "osapi.h"
#include "user_interface.h"
#include "eagle_soc.h"
#include "gpio.h"
#include "ip_addr.h"
#include "mem.h"
#include "espconn.h"

#define SSID "<ssid>"
#define PASSPHRASE "<passprase>"
#define HOSTNAME "timer"
#define NTP0 "0.pool.ntp.org"
#define NTP1 "1.pool.ntp.org"
#define NTP2 "2.pool.ntp.org"
#define TIMEZONE 0 //timezone from -11 to 13
#define DNS "<dns-server-ip>"
//for station IP 192.168.1.33/24 and gateway IP 192.168.1.1/24
#define STATIC_IP_OCTET1 192
#define STATIC_IP_OCTET2 168
#define STATIC_IP_OCTET3 1
#define STATION_IP_OCTET4 33
#define GATEWAY_IP_OCTET4 1
#define NETMASK_OCTET1 255
#define NETMASK_OCTET2 255
#define NETMASK_OCTET3 255
#define NETMASK_OCTET4 0
//relay is on for 5 minutes and off for 55 minutes
#define RELAY_ON 300000 //in ms - 5 minutes
#define RELAY_OFF 3300000 //in ms - 55 minutes
//work hours are from 8 to 17 in TIMEZONE (see above)
#define WORK_HOUR_START 8
#define WORK_HOUR_END 17
#define WORK_HOURS_CHECK 60000 //in ms - every minute
#define RELAY_TIME_ARM_INTERVAL 5000 //in ms - 5 sec
#define SNTP_TIME_ARM_INTERVAL 3000 //in ms - 3 sec

uint32 priv_param_start_sec;
os_timer_t relay_on;
os_timer_t relay_off;
os_timer_t sntp_on;
ip_addr_t *dns_ip = NULL;
void turn_relay_on(void *);
void turn_relay_off(void *);
void start_sntp(void *);

void ICACHE_FLASH_ATTR user_rf_pre_init(void) {

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

void ICACHE_FLASH_ATTR user_init(void) {
	if (wifi_set_opmode_current(1)) //station
		os_printf("wifi set mode ok\n");
	else os_printf("wifi set mode failed\n");
	if (wifi_set_phy_mode(PHY_MODE_11G)) //B,G,N
		os_printf("wifi set phy_mode ok\n");
	else os_printf("wifi set phy_mode failed\n");
	char ssid[32] = SSID;
	char password[64] = PASSPHRASE;
	struct station_config stationConf;
	stationConf.bssid_set = 0; //ignore AP MAC
	os_memcpy(&stationConf.ssid, ssid, 32);
	os_memcpy(&stationConf.password, password, 64);
	
	if (wifi_station_set_config_current(&stationConf))
		os_printf("configure wifi ok\n");
	else os_printf("configure wifi failed\n");

	if (wifi_station_set_reconnect_policy(true))
		os_printf("set reconnect policy ok\n");
	else os_printf("set reconnect policy failed\n");
	
	os_timer_disarm(&relay_on);
	os_timer_disarm(&relay_off);
	os_timer_disarm(&sntp_on);
	os_timer_setfn(&relay_on, (os_timer_func_t *)turn_relay_on, NULL);
	os_timer_setfn(&relay_off, (os_timer_func_t *)turn_relay_off, NULL);
	os_timer_setfn(&sntp_on, (os_timer_func_t *)start_sntp, NULL);
	os_timer_arm(&sntp_on, SNTP_TIME_ARM_INTERVAL, 0);
}

void ICACHE_FLASH_ATTR turn_relay_on(void *arg) {
	uint32 current_stamp = 0;
	char * datetime = NULL;
	char * hours = NULL;
	char hrs[3] = { '\0', '\0', '\0'  };
	int h = 0;
	os_timer_disarm(&relay_on);
	current_stamp = sntp_get_current_timestamp();
	if (current_stamp > 0) {
		datetime = sntp_get_real_time(current_stamp);
		if (datetime) hours = os_strstr(datetime, ":");
		if (hours) {
			os_strncpy(hrs, hours - 2, 2);
			h = atoi(hrs);
		}
		else {
			os_printf("datetime: %s\n", datetime); return;
		}
	}
	else {
		os_timer_arm(&relay_on, RELAY_TIME_ARM_INTERVAL, 0);
		os_printf("unable to get time from sntp server\n"); 
		sntp_init();
		return;
	}
	if (h >= 10 && h <= 16) {
		os_timer_arm(&relay_off, RELAY_ON, 0);
		os_printf("turning relay on...\n");
		gpio_output_set(0, BIT0, BIT0, 0); //Set GPIO0 as low - level output.
	}
	else {
		os_timer_arm(&relay_on, WORK_HOURS_CHECK, 0);
		os_printf("current time %s is outside of working hours\n", datetime);
	}
}
void ICACHE_FLASH_ATTR turn_relay_off(void *arg) {
	os_timer_disarm(&relay_off);
	os_timer_arm(&relay_on, RELAY_OFF, 0);
	os_printf("turning relay off...\n");
	gpio_output_set(BIT0, 0, BIT0, 0); //Set GPIO0 as high - level output.
}

void ICACHE_FLASH_ATTR start_sntp(void *arg) {
	os_timer_disarm(&sntp_on);
	if (wifi_station_get_connect_status() == STATION_GOT_IP) {
		os_printf("station got IP\n");
		if (wifi_station_set_hostname(HOSTNAME))
			os_printf("set hostname ok\n");
		else os_printf("set hostname failed\n");

		if (wifi_station_dhcpc_stop())
			os_printf("stop dhcp client ok\n");
		else os_printf("stop dhcp client failed\n");

		struct ip_info ip;
		IP4_ADDR(&ip.ip, STATIC_IP_OCTET1, STATIC_IP_OCTET2, STATIC_IP_OCTET3, STATION_IP_OCTET4);
		IP4_ADDR(&ip.gw, STATIC_IP_OCTET1, STATIC_IP_OCTET2, STATIC_IP_OCTET3, GATEWAY_IP_OCTET4);
		IP4_ADDR(&ip.netmask, NETMASK_OCTET1, NETMASK_OCTET2, NETMASK_OCTET3, NETMASK_OCTET4);

		if (wifi_set_ip_info(STATION_IF, &ip))
			os_printf("set static ip ok\n");
		else os_printf("set static ip failed\n");

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

		if (sntp_set_timezone(TIMEZONE))
			os_printf("set timezone ok\n");
		else os_printf("set timezone failed\n");

		os_printf("sntp 0: %s\n", sntp_getservername(0));
		os_printf("sntp 1: %s\n", sntp_getservername(1));
		os_printf("sntp 2: %s\n", sntp_getservername(2));

		os_timer_arm(&relay_on, 5000, 0);
	}
	else os_timer_arm(&sntp_on, 3000, 0);
}
