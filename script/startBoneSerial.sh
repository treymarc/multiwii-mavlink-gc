#!/bin/bash


if [ 'am335xevm'=`uname -i` ];
then
#do beaglebone pin muxing

# uart 1 /dev/ttyO1
#echo 20 > /sys/kernel/debug/omap_mux/uart1_rxd
#echo 0 > /sys/kernel/debug/omap_mux/uart1_txd
 
# uart 2 /dev/ttyO2 
echo 1 > /sys/kernel/debug/omap_mux/spi0_d0
echo 21 > /sys/kernel/debug/omap_mux/spi0_sclk

fi
        
        

