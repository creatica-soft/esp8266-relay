esp8266-timer-relay is based on $2USD ESP-01s chip with relay from Aliexpress - https://www.aliexpress.com/item/32843645421.html

An example of programming esp8266 wifi chip using ESP8266 Non-OS SDK - simple sntp time-based relay.

The module is configured to connect to a WiFi access point using dynamic or defined static IP, configure DNS and SNTP time servers
and get time from one of SNTP servers.

It will then turn relay on and off based on the configured crontab lines: one line for turning relay on, another line - off.

Timer parameters (crontab lines, relay_arm_interval and a timezone) may be updated online from an HTTP server using a text file (not html); for example, http://example.com/timer-parameters.txt with 4 lines:

5 * * * * *

20 * * * * *

1000

12

The first line will turn a relay on 5 seconds after each minute. The second line will turn it back off 20 seconds after each minute. The third line sets relay_arm_interval to 1000ms, basically, to check the cron schedule every second. The fourth line sets the timezone to UTC+12.

Building Instructions

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
