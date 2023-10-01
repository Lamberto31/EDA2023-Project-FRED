#!/bin/bash

mac="00:22:04:01:17:C7"

# Check if rfcomm connection is defined for HC-05 mac address
if ! rfcomm | grep -q $mac; then
    echo "Connection not defined, executing rfcomm..."
    rfcomm bind 1 $mac
    # Check again to see if command has been correctly executed
    if ! rfcomm | grep -q $mac; then
        echo "Connection not defined, try again with sudo permission, exiting..."
        exit 1
    fi
fi
echo "Executing python script..."
python3 FRED_data_transmission.py