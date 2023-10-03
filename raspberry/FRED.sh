#!/bin/bash

# Usage and help definition
usage="$(basename "$0") [-h] -- program that check if rfcomm connection is defined and then execute python script.\n
    \tIf a connection is not defined, you need to execute this script with sudo permission and then execute it again without sudo permission.\n
    \tIf a connection is already defined you can execute this script without sudo permission.\n
    \tOptions:\n
        \t\t-h  show this help text"

# Check if option -h is passed
if [ "$1" == "-h" ]; then
  echo -e $usage
  exit 0
fi

# Mac of HC-05 bluetooth module
mac="00:22:04:01:17:C7"

# Check if rfcomm connection is defined for HC-05 mac address
if ! rfcomm | grep -q $mac; then
    # If not defined execute rfcomm bind
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
    echo "Connection defined!"
fi

# Check if script is executed with sudo permission
    if [ "$EUID" -eq 0 ]; then
        echo "Please run as pi to execute python script, exiting..."
        exit 1
    fi

echo "Executing python script..."
python3 FRED_data_transmission.py