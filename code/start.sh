#!/bin/bash

# 等待X伺服器準備好
while [ ! -e /tmp/.X11-unix/X0 ]; do
    sleep 1
done

# 等待使用者會話完全啟動
until xhost >/dev/null 2>&1; do
    sleep 1
done

# 設置必要的環境變量
export DISPLAY=:0
export XAUTHORITY=/home/user/.Xauthority
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus

# 啟動終端並保持打開
/usr/bin/gnome-terminal --title='start code' -- bash -c '/home/user/code/start-code.py; exec bash'