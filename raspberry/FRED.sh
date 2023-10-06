#!/bin/bash

# USAGE AND ARGUMENTS
# Usage and help definition
usage="Usage:\n
        \t$(basename "$0") [options]\n
            \tProgram that check if rfcomm connection is defined and then execute python script:\n
            \t- If a connection is not defined, you need to execute this script with sudo permission and then execute it again without sudo permission.\n
            \t- If a connection is already defined you can execute this script without sudo permission.\n
    Options:\n
        \t-h, --help\t\t show this help text\n
        \t-d, --debug {0,1,2}\t enable debug mode for python script, 0 for none, 1 for default, 2 for full\n
        \t-w, --wifi {0,1}\t enable wifi mode for python script, 0 for disable, 2 for enable\n
    Note:\n
        \tIf you want to pass optional arguments it's necessary to do with space! See the examples\n
    Examples:\n
        \t./FRED.sh -d 2\t\t Correct!\n
        \t./FRED.sh -d 2 -w 1\t Correct!\n
        \t./FRED.sh -d2\t\t Wrong!\n"

# Default arguments
HELP=0
DEBUG=1
WIFI=1

# Parse arguments
POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      HELP=true
      shift # past argument
      ;;
    -d|--debug)
      DEBUG="$2"
      shift # past argument
      shift # past value
      ;;
    -w|--wifi)
      WIFI="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1") # save positional arg
      shift # past argument
      ;;
  esac
done
set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters

# Check if option -h is passed
if [ "$HELP" == "true" ]; then
  echo -e $usage
  exit 0
fi

# MAIN PROGRAMS
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
    rfcomm bind 1 $mac

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
python3 FRED_data_transmission.py -d $DEBUG -w $WIFI