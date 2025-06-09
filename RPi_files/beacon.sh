#!/bin/bash

sudo hciconfig hci0 up
sudo hciconfig hci0 leadv 0

python3 path/main.py

[Unit]
Description=Beacon_Service
After=bluetooth.target
Requires=bluetooth.target

[Service]
ExecStart=/home/pi/beacon.sh
Restart=on-failure
User=pi
WorkingDirectory=/home/pi
StandardOutput=inherit
StandardError=inherit

[Install]
WantedBy=multi-user.target




