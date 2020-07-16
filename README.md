esp8266-timer-relay is based on $2USD ESP-01s chip with relay from Aliexpress - https://www.aliexpress.com/item/32843645421.html

## simple sntp time-based relay with OTA firmware update for RTOS flavor

user_main.c is an example of programming esp8266 wifi chip using ESP8266 Non-OS SDK.

timer.c is an example of programming esp8266 wifi chip using ESP8266 RTOS SDK.

The module is configured to connect to a WiFi access point using dynamic or defined static IP, configure DNS and SNTP time servers
and get time from one of SNTP servers.

It will then turn relay on and off based on the configured crontab lines: one line for turning relay on, another line - off.

Timer parameters (crontab lines, relay_arm_interval and a timezone) may be updated online from an HTTP server using a text file (not html); for example, http://example.com/timer-parameters.txt with 4 lines:

```
5 * * * * *
20 * * * * *
1000
12
```

The first line will turn a relay on 5 seconds after each minute. The second line will turn it back off 20 seconds after each minute. The third line sets relay_arm_interval to 1000ms, basically, to check the cron schedule every second. The fourth line sets the timezone to UTC+12.

For timer.c the last parameter TZ is a string in TZ format (see tzset() for more into); for example, NZST-12NZDT-13,M10.1.0,M3.3.0

## Building Instructions for Non-OS SDK

1. Review and update defines

```
wget https://github.com/espressif/ESP8266_NONOS_SDK/archive/v3.0.3.tar.gz
tar -zxf ESP8266_NONOS_SDK-v3.0.3.tar.gz
mkdir -p ESP8266_NONOS_SDK-3.0.3/timer/user
cp user_main.c ESP8266_NONOS_SDK-3.0.3/timer/user
cp ESP8266_NONOS_SDK-3.0.3/examples/IoT_Demo/Makefile ESP8266_NONOS_SDK-3.0.3/timer
```

2. Comment out "driver" lines in Makefile above

```
cp -r ESP8266_NONOS_SDK-3.0.3/examples/IoT_Demo/include ESP8266_NONOS_SDK-3.0.3/timer/
cp SP8266_NONOS_SDK-3.0.3/examples/IoT_Demo/user/Makefile ESP8266_NONOS_SDK-3.0.3/timer/user 
```

3. Install esp-open-sdk first for crosscompiling: https://github.com/pfalcon/esp-open-sdk

```
export PATH=~/esp-open-sdk/xtensa-lx106-elf/bin:$PATH
```

4. Compile user1.bin for esp-01s with `make COMPILE=gcc BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2`

5. Compile user2.bin for esp-01s with `make clean; make COMPILE=gcc BOOT=new APP=2 SPI_SPEED=40 SPI_MODE=0 SPI_SIZE_MAP=2`

6. Install esptool from https://github.com/espressif/esptool

7. Power off, ground GPIO0 before flashing, power on

Power cycle after each flash!

8. Flash boot.bin first:

```
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x00000 ../ESP8266_NONOS_SDK-3.0.3/bin/boot_v1.7.bin
```

9. Flash user1.bin:

```
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x01000 ../ESP8266_NONOS_SDK-3.0.3/bin/upgrade/user1.1024.new.2.bin
```

10. Flash redundant image user2.bin:

```
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0x81000 ../ESP8266_NONOS_SDK-3.0.3/bin/upgrade/user2.1024.new.2.bin
```

11. Flash blank.bin and esp_init_data_default.bin:

```
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfb000 ../ESP8266_NONOS_SDK-3.0.3/bin/blank.bin
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfc000 ../ESP8266_NONOS_SDK-3.0.3/bin/esp_init_data_default_v08.bin
./esptool.py -p /dev/ttyAMA0 -b 115200 write_flash 0xfe000 ../ESP8266_NONOS_SDK-3.0.3/bin/blank.bin
```

12. Power off, unground GPIO0 for normal operation, power on

13. Run terminal emulator to see the os_printf() messages:

```
miniterm --raw /dev/ttyAMA0 74880
```

## Building Instructions for RTOS SDK

Install toolchain for crosscompiling (may not be easy for arm, old esp-open-sdk toolchain works fine despite of warnings):
 * supported toolchain version is 1.22.0-92-g8facf4c
 * supported compiler version 5.2.0

```
export PATH=~/esp-open-sdk/xtensa-lx106-elf/bin:$PATH
cd ~
wget https://github.com/espressif/ESP8266_RTOS_SDK/archive/v3.1.2.tar.gz
tar -zxf ESP8266_RTOS_SDK-v3.1.2.tar.gz
mkdir -p ESP8266_RTOS_SDK-3.1.2/timer/main
```

RTOS SDK Manual at https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/index.html

Review and update defines in timer.c

```
cp timer.c ESP8266_RTOS_SDK-3.1.2/timer/main
cp ESP8266_RTOS_SDK-3.1.2/examples/get-started/project-template/Makefile ESP8266_RTOS_SDK-3.1.2/timer
touch ESP8266_RTOS_SDK-3.1.2/timer/main/component.mk
export IDF_PATH=~/ESP8266_RTOS_SDK-3.1.2
make menuconfig # to produce sdkconfig
make
```

Power off, ground GPIO0 before flashing, power on

```
make flash
```

Power off, unground GPIO0 for normal operation, power on

Run terminal emulator to see the ESP_LOGI(TAG,) messages:

```
miniterm --raw /dev/ttyAMA1 74880
```
