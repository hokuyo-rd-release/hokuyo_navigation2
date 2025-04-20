#!/bin/bash

echo "Closing websocket!"
pkill -f websocket
sleep 1s
echo "Closing flask!"
pkill -f server.py
sleep 1s
echo "Closing roscore terminal!"
pkill -f roscore
sleep 1s

echo "rosnode kill all!"
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh

exit 0
