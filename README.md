esp8266-timer-relay is based on $2USD ESP-01s chip with relay from Aliexpress - https://www.aliexpress.com/item/32843645421.html

An example of programming esp8266 wifi chip using ESP8266 Non-OS SDK - simple sntp time-based relay.

The module is configured to connect to a WiFi access point using dynamic or defined static IP, configures DNS and SNTP time servers
and gets time from one of SNTP servers.

It will then turn relay on and off based on the configured crontab lines: one line for turning relay on, another line - off.

Timer parameters (crontab lines and a timezone) may be updated online from an HTTP server using a text file (not html); for example, http://example.com/timer-parameters.txt with 3 lines:

5 * * * * *

20 * * * * *

12

The above lines will turn a relay on 5 seconds after each minute and turn it back off 20 seconds after each minute. It will also change the timezone to UTC+12.
