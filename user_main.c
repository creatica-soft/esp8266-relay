/*
Review and update defines

mkdir -p ESP8266_NONOS_SDK-2.2.1/timer/user
cp user_main.c ESP8266_NONOS_SDK-2.2.1/timer/user
cp ESP8266_NONOS_SDK-2.2.1/examples/IoT_Demo/Makefile ESP8266_NONOS_SDK-2.2.1/timer
cp -r ESP8266_NONOS_SDK-2.2.1/examples/IoT_Demo/include ESP8266_NONOS_SDK-2.2.1/timer/
cp SP8266_NONOS_SDK-2.2.1/examples/IoT_Demo/user/Makefile ESP8266_NONOS_SDK-2.2.1/timer/user 

Install esp-open-sdk first for crosscompiling!

export PATH=/home/esp-open-sdk/xtensa-lx106-elf/bin:$PATH

Compile user1.bin for esp-01s with "make COMPILE=gcc BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2"
Compile user2.bin for esp-01s with "make clean; make COMPILE=gcc BOOT=new APP=2 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2"

Power off, ground GPIO0 before flashing, power on

Power cycle after each flash!

Flash boot.bin first:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x00000 esp-01s/boot.bin
Flash user1.bin:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x01000 ../ESP8266_NONOS_SDK-2.2.1/bin/upgrade/user1.1024.new.2.bin
Flash redundant image user2.bin:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x81000 ../ESP8266_NONOS_SDK-2.2.1/bin/upgrade/user2.1024.new.2.bin
Flash blank.bin and esp_init_data_default.bin:
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfb000 esp-01s/blank.bin
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfc000 esp-01s/esp_init_data_default_v08.bin
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfe000 esp-01s/blank.bin

Power off, unground GPIO0 for normal operation, power on

Run terminal emulator to see the os_printf() messages:
miniterm --raw /dev/ttyAMA0 74880
*/
#include "ets_sys.h"
#include "sntp.h"
#include "osapi.h"
#include "user_interface.h"
#include "gpio.h"
#include "ip_addr.h"
#include "mem.h"
#include "espconn.h"

#define SSID "ssid"
#define PASSPHRASE "password"
#define HOSTNAME "timer-relay"
#define NTP0 "0.pool.ntp.org"
#define NTP1 "1.pool.ntp.org"
#define NTP2 "2.pool.ntp.org"
#define DNS "8.8.8.8"
#define HTTP_UPDATE true //update timer parameters from WEB_SERVER
#define HTTP_UPDATE_INTERVAL 3600000 //check for new timer parameters every ms
#define WEB_SERVER "example.com"
#define WEB_SERVER_PORT "80"
#define WEB_SERVER_PATH "/timer.txt"
#define HTTP_METHOD "GET"
#define HTTP_VERSION "HTTP/1.1"
#define USER_AGENT "esp8266"
//delimiter char between values in http:/example.com/timer.txt: 
//relay_on_ms:relay_off_ms:work_hour_start:work_hour_end:tz
#define VALUE_DELIMITER_CHAR ":"
#define USE_STATIC_IP true
#define STATIC_IP "192.168.1.33"
#define NETMASK "255.255.255.0"
#define GATEWAY_IP "192.168.1.1"
//define initial timer parameters, which will become permanent if HTTP_UPDATE false
//relay is on for 5 minutes and off for 55 minutes
#define RELAY_ON 300000 //in ms - 5 minutes
#define RELAY_OFF 3300000 //in ms - 55 minutes
//work hours are from 8 to 17 in TIMEZONE (see above)
#define WORK_HOUR_START 8
#define WORK_HOUR_END 17
//timezone from -11 to 13
#define TIMEZONE 0
//define other static parameters
#define WORK_HOURS_CHECK 60000 //in ms - every minute
#define RELAY_TIME_ARM_INTERVAL 5000 //in ms - 5 sec
#define SNTP_TIME_ARM_INTERVAL 3000 //in ms - 3 sec

uint32 priv_param_start_sec;
os_timer_t relay_on;
os_timer_t relay_off;
os_timer_t sntp_on;
os_timer_t http_update;
int relay_on_ms = RELAY_ON, relay_off_ms = RELAY_OFF, work_hour_start = WORK_HOUR_START, work_hour_end = WORK_HOUR_END, tz = TIMEZONE;

void turn_relay_on(void *);
void turn_relay_off(void *);
void start_sntp(void *);
void http_update_cb(void *);
void connect_cb(void *);//*arg is a pointer to espconn
void reconnect_cb(void *, sint8);//*arg is a pointer to espconn
void disconnect_cb(void *);
void parse_body(char *); //parse http body
void receive_cb(void *, char *, unsigned short);
void sent_cb(void *);
void web_server_found(const char *, ip_addr_t *, void *);


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
	os_timer_disarm(&http_update);
	os_timer_setfn(&relay_on, (os_timer_func_t *)turn_relay_on, NULL);
	os_timer_setfn(&relay_off, (os_timer_func_t *)turn_relay_off, NULL);
	os_timer_setfn(&sntp_on, (os_timer_func_t *)start_sntp, NULL);
	os_timer_setfn(&http_update, (os_timer_func_t *)http_update_cb, NULL);
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
			os_printf("datetime: %s\n", datetime); 
			return;
		}
	}
	else {
		os_timer_arm(&relay_on, RELAY_TIME_ARM_INTERVAL, 0);
		os_printf("unable to get time from sntp server\n"); 
		sntp_init();
		return;
	}
	if (h >= work_hour_start && h <= work_hour_end) {
		os_timer_arm(&relay_off, relay_on_ms, 0);
		os_printf("current time %s, turning relay on...\n", datetime);
		gpio_output_set(0, BIT0, BIT0, 0); //Set GPIO0 as low - level output.
	}
	else {
		os_timer_arm(&relay_on, WORK_HOURS_CHECK, 0);
		os_printf("current time %s is outside of working hours\n", datetime);
	}
}
void ICACHE_FLASH_ATTR turn_relay_off(void *arg) {
	uint32 current_stamp = 0;
	char * datetime = NULL;
	current_stamp = sntp_get_current_timestamp();
	if (current_stamp > 0) {
		datetime = sntp_get_real_time(current_stamp);
		if (datetime) os_printf("current time %s, ", datetime);
	}
	os_timer_disarm(&relay_off);
	os_timer_arm(&relay_on, relay_off_ms, 0);
	os_printf("turning relay off...\n");
	gpio_output_set(BIT0, 0, BIT0, 0); //Set GPIO0 as high - level output.
}

void ICACHE_FLASH_ATTR start_sntp(void *arg) {
	os_timer_disarm(&sntp_on);
	if (wifi_station_get_connect_status() == STATION_GOT_IP) {
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

		if (HTTP_UPDATE) {
			http_update_cb(NULL);
			os_timer_arm(&http_update, HTTP_UPDATE_INTERVAL, 0);
			os_printf("set timer parameters update over http interval to %dms\n", HTTP_UPDATE_INTERVAL);
		}
	} else os_timer_arm(&sntp_on, SNTP_TIME_ARM_INTERVAL, 0);
}
void ICACHE_FLASH_ATTR http_update_cb(void * arg) {
	//get parameters relay_on_ms, relay_off_ms, work_hour_start, work_hour_end from the web server
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
		break;
	case ESPCONN_INPROGRESS:
		os_printf("gethostbyname: waiting for DNS server response\n");
		break;
	case ESPCONN_ARG:
		os_printf("gethostbyname error: wrong argument\n");
		break;
	default:
		os_printf("gethostbyname error: %d\n", err);
		break;
	}
}
void ICACHE_FLASH_ATTR connect_cb(void * arg) {
	struct espconn * con = (struct espconn *)arg;
	if (!con) return;
	/*remot_info * server = NULL;
	espconn_get_connection_info(con, &server, 0);
	if (server) {
		os_printf("%d.%d.%d.%d:%d", server->remote_ip[0], server->remote_ip[1], server->remote_ip[2], server->remote_ip[3], server->remote_port);
		switch (server->state) {
		case ESPCONN_NONE:
			os_printf(" - not connected\n");
			break;
		case ESPCONN_WAIT:
			os_printf(" - waiting\n");
			break;
		case ESPCONN_LISTEN:
			os_printf(" - listening\n");
			break;
		case ESPCONN_CONNECT:
			os_printf(" - connected\n");
			break;
		case ESPCONN_WRITE:
			os_printf(" - sending\n");
			break;
		case ESPCONN_READ:
			os_printf(" - receiving\n");
			break;
		case ESPCONN_CLOSE:
			os_printf(" - closed\n");
			break;
		default:
			os_printf(" - unknown state\n");
		}
	} else os_printf("connect_cb: espconn_get_connection_info() error\n");*/
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
//parse text to extract relay_on_ms, relay_off_ms, work_hour_start, work_hour_end, tz int values
void ICACHE_FLASH_ATTR parse_body(char * body) {
	char * del_pos = NULL;
	char val[8];
	int l, i = 0;
	//os_printf("body:\n%s\n", body);
	do {
		del_pos = os_strstr(body, VALUE_DELIMITER_CHAR);
		if (del_pos) 
			l = del_pos - body;
		else l = os_strlen(body);
		if (l >= sizeof(val)) {
			os_printf("value is too large %d\n", l);
			continue;
		} //else os_printf("value %d length %d\n", i, l);
		os_bzero(val, sizeof(val));
		os_strncpy(val, body, l);
		switch (i) {
		case 0:
			relay_on_ms = atoi(val);
			os_printf("set RELAY_ON timer to %dms\n", relay_on_ms);
			break;
		case 1:
			relay_off_ms = atoi(val);
			os_printf("set RELAY_OFF timer to %dms\n", relay_off_ms);
			break;
		case 2:
			work_hour_start = atoi(val);
			os_printf("set WORK_HOUR_START to %d\n", work_hour_start);
			break;
		case 3:
			work_hour_end = atoi(val);
			os_printf("set WORK_HOUR_END to %d\n", work_hour_end);
			break;
		case 4:
			tz = atoi(val);
			os_printf("set TIMEZONE to %d\n", tz);
			break;
		default:
			os_printf("too many values in http response - ignored\n");
			return;
		}
		body = del_pos + 1;
		i++;
	} while (del_pos);
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
				//else os_printf("html body length %d, content length %d\n", body_len, content_len);
			} else os_printf("receive_cb error: http_status %d\n", http_status);
		} else parse_body(buf);
		os_free(buf);
	} else os_printf("receive_cb: http response is empty\n");
	if (tz != TIMEZONE) {
		if (sntp_set_timezone(tz))
			os_printf("sntp set timezone %d ok\n", tz);
		else os_printf("sntp set timezone %d failed\n", tz);
	}
	os_timer_arm(&relay_on, RELAY_TIME_ARM_INTERVAL, 0);
}
void web_server_found(const char * name, ip_addr_t * ip, void * args) {
	if (ip) {
		struct espconn *con = (struct espconn *)args;
		os_memcpy(con->proto.tcp->remote_ip, ip, 4);
		os_printf("web server %s ip is %d.%d.%d.%d\n", name, con->proto.tcp->remote_ip[0], con->proto.tcp->remote_ip[1], con->proto.tcp->remote_ip[2], con->proto.tcp->remote_ip[3]);

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
