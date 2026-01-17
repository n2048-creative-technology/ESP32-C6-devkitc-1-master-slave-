#!/usr/bin/env sh
for p in $(ls /dev/ttyACM*); do   
    pio run -e master -t upload --upload-port "$p"; 
done
