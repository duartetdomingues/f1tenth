#!/bin/bash
MAC="40:8E:2C:A0:F8:0D"
bluetoothctl << EOF
remove $MAC
scan on
pair $MAC
trust $MAC
connect $MAC
EOF