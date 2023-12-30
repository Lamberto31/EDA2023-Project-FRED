import serial
import time
import requests
from decouple import config
import argparse
import csv
import datetime
import os
import signal

# INITIAL DEFINITIONS
# Functionalities active/disabled

# DEBUG can be 3 value: None, Default, Full. Can be passed as argument (int from 0 to 2)
# None: none
# Default: only essential
# Full: all, also the BDT message
DEBUG = "Default"

# VIEW_DATA can be 2 value: False, True. Can be passed as argument (int from 0 to 1). Works only if DEBUG is not None
# False: disable data view
# True: enable data view
VIEW_DATA = True

# WIFI can be 2 value: False, True. Can be passed as argument (int from 0 to 1)
# False: disable wifi connection
# True: enable wifi connection
WIFI = True

# PARAMS can be 2 value: False, True, Can be passed as argument (int from 0 to 1)
# False: disable params processing
# True: enable params processing
PARAMS = False

# Parameters Definition
PERIOD_SERVER = 15 # seconds
# Status strings
STATUS_DISCONNECTED = "Disconnected"
STATUS_CONNECTED = "Connected"

STATUS_SETUP = "Setup"

STATUS_FREE = "Free walking"
STATUS_READING = "Reading"
STATUS_EXPLORATION = "Exploration"
STATUS_DATA_TRANSMISSION = "Data transmission"

STATUS_IDLE = "Idle"
STATUS_INPUT_MAX = "Input max"
STATUS_INPUT_0 = "Input 0"
STATUS_STOP = "Stop"

STATUS_UNKNOWN = "Unknown"

# Info strings
INFO_CUSTOM_DISTANCE = "Custom distance"

# Get secret values from .env file
API_KEY = config('API_KEY')
CHANNEL_ID = config('CHANNEL_ID')

# Parser
parser = argparse.ArgumentParser("FRED_data_transmission", description= "Script used to receive BDT from FRED and send them to remote server.")
parser.add_argument("--debug", "-d",  help="An integer that define a debug level:\n\
                    0 is none\n\
                    1 is default\n\
                    2 is full.\n\
                    If not passed it will be a code-defined value\
                    ", type=int, choices=[0, 1, 2], default=1)
parser.add_argument("--viewdata", "-v", help="Enable/disable tabular data visualization:\
                    0 is disable\
                    1 is enable\
                    ", type=int, choices=[0, 1], default=1)
parser.add_argument("--wifi", "-w", help="Enable/disable wifi connection:\
                    0 is disable\
                    1 is enable\
                    ", type=int, choices=[0, 1], default=1)
parser.add_argument("--params", "-p", help="Enable/disable params processing:\
                    0 is disable\
                    1 is enable\
                    ", type=int, choices=[0, 1], default=0)
args = parser.parse_args()

# Connection status
# Check if the connection is established
connected = False
# Check if the connection is lost (after a first connection!)
disconnected = False

# FUNCTIONS DEFINITION

# Transform status number in status string
def getStatusString(statusNumber):
    if statusNumber == "0":
        return STATUS_SETUP
    elif statusNumber == "1":
        return STATUS_FREE
    elif statusNumber == "2":
        return STATUS_EXPLORATION
    elif statusNumber == "3":
        return STATUS_DATA_TRANSMISSION
    elif statusNumber == "4":
        return STATUS_READING
    else:
        return STATUS_UNKNOWN

# Insert received data in stored data
def insertDataInDict(recvData):
    decoded = recvData.decode('utf-8')
    clean = decoded[0:-2]
    data = clean.split(":")
    if data[0] == "Input":
        dataDict["field1"] = data[1]
    elif data[0] == "Measures":
        vector = data[1].split(",")
        dataDict["field2"] = vector[0]
        dataDict["field3"] = vector[1]
    elif data[0] == "State":
        vector = data[1].split(",")
        dataDict["field4"] = vector[0]
        dataDict["field5"] = vector[1]
    elif data[0] == "Covariance":
        vector = data[1].split(",")
        dataDict["field6"] = vector[0]
        dataDict["field7"] = vector[1]
    elif data[0] == "Status":
        dataDict["status"] = getStatusString(data[1])

# Conditional print
def debugStamp(str, level="Default"):
    if DEBUG == "None":
        return
    elif DEBUG == "Full":
        print(str)
    elif DEBUG == level:
        print(str)

# Print data in tabular format
def stampData(dataToPrint):
    if ((DEBUG == "Default" or DEBUG == "Full") and VIEW_DATA):
        print("\nDATA")
        print("N.\tcreated_at\tfield1\tfield2\tfield3\tfield4\tfield5\tfield6\tfield7\tfield8\tstatus\n")
        for i in range(0, len(dataToPrint)):
            print(str(i + 1) + "\t" + str(dataToPrint[i]["created_at"]) + "\t" + str(dataToPrint[i]["field1"]) + "\t" + str(dataToPrint[i]["field2"]) + "\t" + str(dataToPrint[i]["field3"]) + "\t" + str(dataToPrint[i]["field4"]) + "\t" + str(dataToPrint[i]["field5"]) + "\t" + str(dataToPrint[i]["field6"]) + "\t" + str(dataToPrint[i]["field7"]) + "\t" + str(dataToPrint[i]["field8"]) + "\t" + str(dataToPrint[i]["status"]) + "\n")

# Print matrix in tabular format
def stampMatrix(metadata, data):
    # Process metadata
    dimensions = metadata.split("=")[1]
    rows = int(dimensions.split("x")[0])
    columns = int(dimensions.split("x")[1])
    print(metadata)
    # Process data
    data = data.split(":")[1].split(",")
    if ((DEBUG == "Default" or DEBUG == "Full")):
        for i in range(0, rows):
            for j in range(0, columns):
                print(data[i * columns + j], end=" ")
            print()

# Handle CTRL+C
def interruptHandler(sig, frame):
    ser.close()  # Close serial connection, this will cause error and so the script will stop but safely
    signal.signal(signal.SIGINT, signal.default_int_handler)
    print("Interrupt received")
    print("If you want to close the script immediately send interrupt again")

# Interpret input arguments
# DEBUG
def interpretDebugArguments():
    if args.debug == 0:
        DEBUG = "None"
    elif args.debug == 1:
        DEBUG = "Default"
    elif args.debug == 2:
        DEBUG = "Full"
    return DEBUG
# VIEWDATA
def interpretViewDataArguments():
    if args.viewdata == 0:
        VIEW_DATA = False
    elif args.viewdata == 1:
        VIEW_DATA = True
    return VIEW_DATA
# WIFI
def interpretWifiArguments():
    if args.wifi == 0:
        WIFI = False
    elif args.wifi == 1:
        WIFI = True
    return WIFI
# PARAMS
def interpretParamsArguments():
    if args.params == 0:
        PARAMS = False
    elif args.params == 1:
        PARAMS = True
    return PARAMS

# INITIAL CONFIGURATION

# Print closing instructions
print("Press CTRL+C or send SIGINT to safely close the script")

# Interpret input arguments
DEBUG = interpretDebugArguments()
WIFI = interpretWifiArguments()
VIEW_DATA = interpretViewDataArguments()
PARAMS = interpretParamsArguments()

# Print initial configuration
print("Functionalities configuration")
print("DEBUG: " + DEBUG + " (" + str(args.debug) + ")")
print("VIEW_DATA: " + str(VIEW_DATA) + " (" + str(args.viewdata) + ")")
print("WIFI: " + str(WIFI) + " (" + str(args.wifi) + ")")
print("PARAMS: " + str(PARAMS) + " (" + str(args.params) + ")")

# Serial connection configuration
ser = serial.Serial(
    port='/dev/rfcomm1',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

# Check if the serial connection is open, if not stop execution
if (ser.isOpen()):
    debugStamp("Serial connection started")
else:
    debugStamp("Serial connection not started, try again")
    exit()

# Init dictionary that contains data (declared as field as in the remote server)
dataDict = {
    "created_at": 0,
    "field1": 0,
    "field2": 0,
    "field3": 0,
    "field4": 0,
    "field5": 0,
    "field6": 0,
    "field7": 0,
    "field8": 0,
    "status": STATUS_UNKNOWN
    }

# Init list of dataToWrite and dataToSend
dataToWrite = []
dataToSend = []

# Init json to send
jsonDict = {}
jsonDict["write_api_key"] = API_KEY
jsonDict["updates"] = dataToSend

# Info dictionary
infoDict = {
    INFO_CUSTOM_DISTANCE: 0
}


# CSV file
# Init csv file
# Create file name
timestamp = datetime.datetime.now().replace(microsecond=0).isoformat()
csvFileName = "FRED_log_" + timestamp + ".csv"

# Create file and write header
csvFile = open(os.path.join("./logs", csvFileName), mode='w')
csvWriter = csv.DictWriter(csvFile, fieldnames=dataDict.keys())
csvWriter.writeheader()
csvFile.flush()

# Init last execution time
lastSendToServer = time.time()

# PARAMS PROCESSING
if PARAMS:
    def getParamsStatusString(statusNumber):
        if statusNumber == "0":
            return STATUS_SETUP
        elif statusNumber == "1":
            return STATUS_IDLE
        elif statusNumber == "2":
            return STATUS_INPUT_MAX
        elif statusNumber == "3":
            return STATUS_INPUT_0
        elif statusNumber == "4":
            return STATUS_STOP
        else:
            return STATUS_UNKNOWN
    # Init dictionary that contains data received with PARAMS
    paramsData = {
        "attempt": 1,
        "currentTime": 0,
        "distance": 0,
        "speed": 0,
        "stopTime": 0,
        "status": STATUS_UNKNOWN
        }
    # Define function that fill params dictionary
    def insertParamsInDict(recvParams):
        decoded = recvParams.decode('utf-8')
        clean = decoded[0:-2]
        data = clean.split(":")
        if "distance" in data[0].lower():
            paramsData["distance"] = data[1]
        elif "speed" in data[0].lower():
            paramsData["speed"] = data[1]
        elif "current" in data[0].lower():
            paramsData["currentTime"] = data[1]
        elif "stop" in data[0].lower():
            paramsData["stopTime"] = data[1]
    # Create csv params file and write header
    paramsCsvFileName = "FRED_params_" + timestamp + ".csv"
    paramsCsvFile = open(os.path.join("./logs", paramsCsvFileName), mode='w')
    paramsCsvWriter = csv.DictWriter(paramsCsvFile, fieldnames=paramsData.keys())
    paramsCsvWriter.writeheader()
    paramsCsvFile.flush()
    # Define function that write a row in csv if enough data are present
    def writeParamsCsv(statusString):
        if statusString == STATUS_STOP:
            paramsCsvWriter.writerow({"attempt": paramsData["attempt"], "currentTime": paramsData["currentTime"], "distance": paramsData["distance"], "speed": paramsData["speed"], "stopTime": paramsData["stopTime"], "status": paramsData["status"]})
            debugStamp("Attempt " + str(paramsData["attempt"]) + " finished")
            paramsData["attempt"] += 1
        else:
            paramsCsvWriter.writerow({"attempt": paramsData["attempt"], "currentTime": paramsData["currentTime"], "distance": paramsData["distance"], "speed": paramsData["speed"], "status": paramsData["status"]})
        paramsCsvFile.flush()
    # Print params in tabular format
    def stampParams(first):
        if ((DEBUG == "Default" or DEBUG == "Full") and VIEW_DATA):
            if first:
                print("\nPARAMS")
                print("attempt\tcurrentTime\tdistance\tspeed\t\tstopTime\tstatus\n")
            print(str(paramsData["attempt"]) + "\t" + str(paramsData["currentTime"]) + "\t\t" + str(paramsData["distance"]) + "\t\t" + str(paramsData["speed"]) + "\t\t" + str(paramsData["stopTime"]) + "\t\t" + str(paramsData["status"]) + "\n")


# MAIN LOOP: receive data from Bluetooth and send to remote server via WiFi
debugStamp("Starting main loop")
while True:
    # RECEIVE DATA FROM BLUETOOTH
    # Check if there are incoming data
    try:
        if not disconnected and ser.in_waiting > 0:
            # Check if the connection is established
            if not connected:
                debugStamp("Bluetooth connection established")
                connected = True
                if (WIFI):
                    debugStamp("Sending connected status to remote server")
                    r = requests.post("https://api.thingspeak.com/update.json", json = {"api_key": API_KEY, "status": STATUS_CONNECTED})
                    debugStamp(str(r.status_code) + " " + str(r.reason))
                # Handle CTRL+C
                signal.signal(signal.SIGINT, interruptHandler)
            # Read data
            recv = ser.readline()
            # If contains "DATA" or "ESTIMATE" it's a DATA/ESTIMATE messagge
            if "DATA" in str(recv) or "ESTIMATE" in str(recv):
                if "DATA" in str(recv):
                    messageNumber = 4
                    debugStamp("New BDT message: DATA")
                elif "ESTIMATE" in str(recv):
                    messageNumber = 5
                    debugStamp("New BDT message: ESTIMATE")
                debugStamp(str(recv, 'utf-8'), "Full")
                messageCounter = 0
                while True:
                    recv = ser.readline()
                    messageCounter += 1
                    debugStamp(str(recv, 'utf-8'), "Full")
                    insertDataInDict(recv)
                    # If it's the last message, add timestamp and append to dataToWrite and dataToSend
                    if messageCounter == messageNumber:
                        debugStamp("Last message received")
                        dataDict["created_at"] = int((time.time()*1000))  # milliseconds
                        dataDict["field8"] = infoDict[INFO_CUSTOM_DISTANCE]
                        # Append all data in dataToWrite
                        dataToWrite.append(dataDict.copy())
                        # Append only one data for each second in dataToSend (first one)
                        if (dataToSend and dataToSend[-1]["created_at"] == int(dataDict["created_at"]/1000)):
                            break
                        dataToAdjust = dataDict.copy()
                        dataToAdjust["created_at"] = int(dataToAdjust["created_at"]/1000)
                        dataToSend.append(dataToAdjust.copy())
                        stampData(dataToWrite)
                        break
            # If contains "INFO" it's a INFO messagge
            elif "INFO" in str(recv):
                debugStamp("New BDT message: INFO")
                debugStamp(str(recv, 'utf-8'), "Full")
                info = ser.readline().decode('utf-8')[0:-2].split(":")
                infoDict[str(info[0])] = str(info[1])
                debugStamp(infoDict)
            # If contains "PARAMS" it's a PARAMS messagge
            elif "PARAMS" in str(recv):
                debugStamp("New BDT message: PARAMS")
                debugStamp(str(recv, 'utf-8'), "Full")
                if PARAMS:
                    last = False
                    first = True # Used for stampParams
                    while not last:
                        # Read status and configure message receiving
                        params = ser.readline()
                        statusString = getParamsStatusString(((params.decode('utf-8')[0:-2]).split(":"))[1])
                        debugStamp(statusString, "Full")
                        paramsData["status"] = statusString
                        messageNumber = 3
                        if statusString == STATUS_STOP:
                            messageNumber = 4
                            last = True
                        for i in range(0, messageNumber):
                            params = ser.readline()
                            insertParamsInDict(params)
                            debugStamp(str(params.decode('utf-8')[0:-2]), "Full")
                        stampParams(first)
                        first = False
                        writeParamsCsv(statusString)
            # If contains "MATRIX" it's a MATRIX messagge
            elif "MATRIX" in str(recv):
                debugStamp("New BDT message: MATRIX")
                debugStamp(str(recv, 'utf-8'), "Full")
                # Receive metadata
                recv = ser.readline()
                debugStamp(str(recv, 'utf-8'), "Full")
                metadata = recv.decode('utf-8')[0:-2]
                # Receive data
                recv = ser.readline()
                debugStamp(str(recv, 'utf-8'), "Full")
                data = recv.decode('utf-8')[0:-2]
                stampMatrix(metadata, data)

                    
    except Exception as e:
        debugStamp(e, "Full")
        # If there is an error, handle the closing of the program
        # This is the case when the bluetooth connection is active
        if connected:
            connected = False
            disconnected = True
            debugStamp("Bluetooth connection lost")
            if (WIFI and dataToSend):
                debugStamp("Waiting for data to send and then close the script")
            # If WIFI disabled the script can be closed immediately
            else:
                debugStamp("Closing the script")
                ser.close()
                exit()
        # This is the case when the bluetooth connection is not active, so the script can be closed
        else:
            debugStamp("Bluetooth connection not established, run the script again")
            ser.close()
            exit()
    
    # WRITE AND SEND DATA TO REMOTE SERVER
    # Do it every PERIOD_SERVER seconds and if dataToSend is not empty
    if time.time() - lastSendToServer >= PERIOD_SERVER and dataToSend:
        # Write full data to csv with all data
        csvWriter.writerows(dataToWrite)
        csvFile.flush()

        # Prepare and send data
        if WIFI:
            # Build json to send
            jsonDict["updates"] = dataToSend
            debugStamp(jsonDict, "Full")

            # Send data
            debugStamp("Sending " + str(len(dataToSend)) + " data to remote server")
            r = requests.post("https://api.thingspeak.com/channels/"+ CHANNEL_ID +"/bulk_update.json", json=jsonDict)
            debugStamp(str(r.status_code) + " " + str(r.reason))

        # Reset data
        dataToWrite = []
        dataToSend = []
        jsonDict["updates"] = dataToSend

        # Reset timer
        lastSendToServer = time.time()

        # Close program if disconnected but after sending data
        if disconnected:
            if (WIFI):
                debugStamp("Sending disconnected status to remote server")
                r = requests.post("https://api.thingspeak.com/update.json", json = {"api_key": API_KEY, "status": STATUS_DISCONNECTED})
                debugStamp(str(r.status_code) + " " + str(r.reason))
            debugStamp("Data sent, now the script can be closed")
            ser.close()
            exit()