#/bin/bash

canconfig can0 bitrate 1000000 ctrlmode loopback off

canconfig can0 start

#candump &
