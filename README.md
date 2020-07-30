esp8266-timer-relay is based on $2USD ESP-01s chip with relay from Aliexpress - https://www.aliexpress.com/item/32843645421.html

There are two SDKs available for programming esp8266 chips: traditioinal Non-OS callback-oriented SDK and modern RTOS task-driven one based on freertos.

There are also two approaches to programming the chip: push and pull. 

With the push approach, the switch logic runs somewhere else, externally; for example, on a raspberry pi benefitting from a faster CPU, more RAM and readily available numerous tools, languages and scripts. The esp chip only connects to an access point (or runs its own) and listens for 0 or 1 on a certain network port. 0 will turn the relay off, 1 - on. Simple. 

See simple-switch.c for the details. It has three tasks: one to configure the station once it connects to an AP, another monitors the WiFi status and restarts the station if something goes wrong and the relay_task that listens for 0 or 1 on a UDP port 7777, which can easilly be done using netcat:

```
echo -n 0|nc -un -w 1 192.168.1.100 7777
```

Slightly more advance example is remote-switch.c, which adds two more tasks: firmware update over the air (OTA) and remote logging.

The pull approach incorporates the switch logic into the chip. It creates more network traffic and is limited by memory and cpu resources of the chip as well as the available programming tools. In short, it is more challenging but perhaps, more fun.

Two examples below use the second approach but different SDKs.

## simple sntp time-based relay with OTA firmware update and remote logging for RTOS flavor

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
wget https://github.com/espressif/ESP8266_RTOS_SDK/releases/download/v3.3/ESP8266_RTOS_SDK-v3.3.zip
unzip ESP8266_RTOS_SDK-v3.3.zip
mkdir -p ESP8266_RTOS_SDK/timer/main
```

RTOS SDK Manual at https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/index.html

Review and update defines in timer.c

```
cp timer.c ESP8266_RTOS_SDK/timer/main
cp ESP8266_RTOS_SDK/examples/get-started/project-template/Makefile ESP8266_RTOS_SDK/timer
touch ESP8266_RTOS_SDK/timer/main/component.mk
export IDF_PATH=~/ESP8266_RTOS_SDK
make menuconfig # to produce sdkconfig
make
```

Power off, ground GPIO0 before flashing, power on

```
make flash
```

Power off, unground GPIO0 for normal operation, power on

On a logging server REMOTE_LOGGING_IP:REMOTE_LOGGING_UDP_PORT create a service

```
vi /usr/bin/timer-logging.sh
#!/bin/bash
nc -ulkn 0.0.0.0 6666 >> /var/log/timer.log

chmod 755 /usr/bin/timer-logging.sh

vi /lib/systemd/system/timer-logging.service
[Unit]
Description=Timer Service
Wants=network.target
After=network.target nss-lookup.target

[Service]
#Type=forking
ExecStart=/usr/bin/timer-logging.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target


systemctl daemon-reload
systemctl enable timer-logging
systemctl start timer-logging
systemctl status timer-logging
tail -f /var/log/timer.log
```
Configure logrotate

```
vi /etc/logrotate.d/timer
/var/log/timer.log
{
        su root root
        rotate 7
        daily
        maxsize 1000000
        missingok
        notifempty
        compress
        delaycompress
        sharedscripts
        postrotate
                systemctl restart timer
        endscript
}
```

Test it with

```
logrotate -f /etc/logrotate.d/timer
```

Or set REMOTE_LOGGING to false and rebuild

```
#define REMOTE_LOGGING false
```

Then run terminal emulator to see the ESP_LOGx(TAG,) messages:

```
miniterm --raw /dev/ttyAMA1 74880
```
