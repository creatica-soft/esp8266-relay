#!/bin/bash
STATION_MAC="xx:xx:xx:xx:xx:xx"
TMPFILE="/tmp/station-${STATION_MAC}"
LOGGING_DAEMON_IP="127.0.0.1"
LOGGING_DAEMON_PORT="6666"
DATETIME=`date`
iw wlan0 station dump | grep -A17 ${STATION_MAC} > ${TMPFILE}
RX_BYTES=`grep "rx bytes:" ${TMPFILE} | awk -F":" '{print $2}' | tr -d "\t"`
TX_BYTES=`grep "tx bytes:" ${TMPFILE} | awk -F":" '{print $2}' | tr -d "\t"`
AUTHORIZED=`grep "authorized:" ${TMPFILE} | awk -F":" '{print $2}' | tr -d "\t"`
AUTHENTICATED=`grep "authenticated:" ${TMPFILE} | awk -F":" '{print $2}' | tr -d "\t"`
ASSOCIATED=`grep "associated:" ${TMPFILE} | awk -F":" '{print $2}' | tr -d "\t"`
CONNECTED_TIME=`grep "connected time:" ${TMPFILE} | awk -F":" '{print $2}' | tr -d "\t"`
if [[ "${AUTHORIZED}" == "yes" && "${AUTHENTICATED}" == "yes" && "${ASSOCIATED}" == "yes" ]]; then
        echo "${DATETIME}: engine-room-fan station ${STATION_MAC} is connected for ${CONNECTED_TIME}}, rx ${RX_BYTES} bytes, tx ${TX_BYTES} bytes" | nc -un -w 1 ${LOGGING_DAEMON_IP} ${LOGGING_DAEMON_PORT}
else
        echo "${DATETIME}: engine-room-fan station ${STATION_MAC} is disconnected" | nc -un -w 1 ${LOGGING_DAEMON_IP} ${LOGGING_DAEMON_PORT}
# see gpio tools - connect a beeper to GPIO 21 for audible alarm 
        for ((i=0; i<10; i++)); do
                ~/gpio/gpioset 21 100000
                sleep 0.1s
        done
fi
