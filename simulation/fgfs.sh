#!/bin/sh

FGFS=fgfs

if [ $# != 1 ]
then
echo usage: $0 aircraft
echo choose one from below:
fgfs --show-aircraft
exit
else
aircraft=$1
fi


${FGFS} \
    --aircraft=$aircraft \
    --geometry=400x300 \
    --generic=socket,out,50,127.0.0.1,15501,udp,MAVLink \
    --generic=socket,in,50,127.0.0.1,15500,udp,MAVLink \
    --vc=30 \
    --altitude=10000 \
    --heading=90 \
    --roll=0 \
    --pitch=0 \
    --wind=0@0 \
    --turbulence=0.0 \
    --prop:/sim/frame-rate-throttle-hz=30 \
    --timeofday=noon \
    --shading-flat \
    --fog-disable \
    --disable-specular-highlight \
    --disable-random-objects \
    --disable-panel \
    --disable-horizon-effect \
    --disable-clouds \
    --disable-anti-alias-hud \
    --disable-sound \
    --prop:/engines/engine/running=true \
    --httpd=5400 
