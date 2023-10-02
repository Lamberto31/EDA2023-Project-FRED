#!/bin/bash

mac="00:22:04:01:17:C7"

# Check if rfcomm connection is defined for HC-05 mac address
if ! rfcomm | grep -q $mac; then
    echo "Connection not defined, executing rfcomm..."
    # Check if script is executed with sudo permission
    if [ "$EUID" -ne 0 ]; then
        echo "Please run as root to execute rfcomm bind, exiting..."
        exit 1
    fi
    sudo rfcomm bind 1 $mac
    # Check again to see if command has been correctly executed
    if ! rfcomm | grep -q $mac; then
        echo "Connection not defined, try again with sudo permission, exiting..."
        exit 1
    fi

# Check if script is executed with sudo permission
    if [ "$EUID" -eq 0 ]; then
        echo "Please run as pi to execute python script, exiting..."
        exit 1
    fi

echo "Executing python script..."
python3 FRED_data_transmission.py