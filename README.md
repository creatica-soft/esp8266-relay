esp8266-timer-relay is based on $2USD ESP-01s chip with relay from Aliexpress - https://www.aliexpress.com/item/32843645421.html

An example of programming esp8266 wifi chip using ESP8266 Non-OS SDK - simple sntp time-based relay.

The module is configured to connect to a WiFi access point using defined static IP, configures DNS and SNTP time servers
and set its time from one of SNTP servers.

It will then check if the current time is within the defined work hours and turn the relay on for the defined interval, 
then turn it back off for another defined interval, and so on.

Timer on/off parameters, work hours and a timezone may be set and periodically refreshed from an HTTP server using a single string in a text file (not html) with delimited values; for example, a single line in http://example.com/timer.txt

<timer_on>:<timer_off>:<work_hour_start>:<work_hour_end>:<timezone>
  

