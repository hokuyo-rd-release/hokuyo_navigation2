#!/bin/bash

gnome-terminal -e "ls -l" &
sleep 1 # ターミナルが起動するまで少し待つ
WINDOW_ID=$(wmctrl -l | grep "ls -l" | awk '{print $1}')
if [ -n "$WINDOW_ID" ]; then
  wmctrl -i -r "$WINDOW_ID" -b add,iconified
else
  echo "ウィンドウが見つかりませんでした。"
fi