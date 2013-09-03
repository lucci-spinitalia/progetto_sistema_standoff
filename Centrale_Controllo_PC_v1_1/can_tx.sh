#!/bin/bash
cat /home/root/remote_dev/can_tx | ssh root@192.168.178.28 'cat > /home/root/remote_dev/can_rx'
