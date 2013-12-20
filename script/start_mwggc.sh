#!/bin/sh

# enable uart1
echo BB-UART1 >  /sys/devices/bone_capemgr.8/slots 
# set mode

stty -F /dev/ttyO1  cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts 
#stty -F /dev/ttyACM0  cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts

# wifi
#ip link set wlan0 up
#ifup wlan0

#/home/debian/DEV/multiwii-mavlink-gc/src/udp/mwgc-log -s /dev/ttyO1 > /home/debian/DEV/logMSP.csv 

