#!/usr/bin/env sh
for p in $(ls /dev/ttyACM*); do   
    pio run -e slave -t upload --upload-port "$p"; 
done
