import serial
import time
import requests
from decouple import config
import argparse

# INITIAL DEFINITIONS
# Functionalities active/disabled

# DEBUG can be 3 value: None, Default, Full. Can be passed as argument (int from 0 to 2)
# None: none
# Default: only essential
# Full: all, also the BDT message
DEBUG = "Default"

# Parameters Definition
PERIOD_SERVER = 15 # seconds

# Get secret values from .env file
API_KEY = config('API_KEY')
CHANNEL_ID = config('CHANNEL_ID')

# Parser
parser = argparse.ArgumentParser("FRED_data_transmission", description= "Script used to receive BDT from FRED and send them to remote server.")
parser.add_argument("--debug", "-d",  help="An integer that define a debug level:\n\
                    0 is none\n\
                    1 is default\n\
                    2 is full.\n\
                    If not passed it will be a code-defined value", type=int, choices=[0, 1, 2], default=1)
args = parser.parse_args()

# FUNCTIONS DEFINITION
# Insert received data in stored measures
def insertDataInDict(recvData):
    decoded = recvData.decode('utf-8')
    clean = decoded[0:-2]
    data = clean.split(":")
    if data[0] == "Distance_US":
        measures["field1"] = data[1]
    elif data[0] == "Distance_US_Filtered":
        measures["field2"] = data[1]
    elif data[0] == "Distance_OPT":
        measures["field3"] = data[1]
    elif data[0] == "Rev_per_second":
        measures["field4"] = data[1]
    elif data[0] == "Velocity_US":
        measures["field5"] = data[1]
    elif data[0] == "Velocity_OPT":
        measures["field6"] = data[1]
    elif data[0] == "Velocity_OPT_Filtered":
        measures["field7"] = data[1]

# Conditional print
def debugStamp(str, level="Default"):
    if DEBUG == "None":
        return
    elif DEBUG == "Full":
        print(str)
    elif DEBUG == level:
        print(str)

# Print dataToSend in tabular format
def stampDataToSend():
    if DEBUG == "Default":
        print("\nDATA TO SEND")
        print("N.\tcreated_at\tfield1\tfield2\tfield3\tfield4\tfield5\tfield6\tfield7\n")
        for i in range(0, len(dataToSend)):
            print(str(i + 1) + "\t" + str(dataToSend[i]["created_at"]) + "\t" + str(dataToSend[i]["field1"]) + "\t" + str(dataToSend[i]["field2"]) + "\t" + str(dataToSend[i]["field3"]) + "\t" + str(dataToSend[i]["field4"]) + "\t" + str(dataToSend[i]["field5"]) + "\t" + str(dataToSend[i]["field6"]) + "\t" + str(dataToSend[i]["field7"]) + "\n")

# Interpret input arguments
def interpretDebugArguments():
    if args.debug == 0:
        DEBUG = "None"
    elif args.debug == 1:
        DEBUG = "Default"
    elif args.debug == 2:
        DEBUG = "Full"
    return DEBUG

# INITIAL CONFIGURATION
# Interpret input arguments
DEBUG = interpretDebugArguments()

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

# Init dictionary that contains measures (declared as field as in the remote server)
measures = {
    "created_at": 0,
    "field1": 0,
    "field2": 0,
    "field3": 0,
    "field4": 0,
    "field5": 0,
    "field6": 0,
    "field7": 0
    }

# Init list of dataToSend
dataToSend = []

# Init json to send
jsonDict = {}
jsonDict["write_api_key"] = API_KEY
jsonDict["updates"] = dataToSend

# Init last execution time
lastSendToServer = time.time()

# MAIN LOOP: receive data from Bluetooth and send to remote server via WiFi
debugStamp("Starting main loop")
while True:
    # RECEIVE DATA FROM BLUETOOTH
    # Check if there are incoming data
    if ser.inWaiting() > 0:
        # Read data
        recv = ser.readline()
        # If contains "START" it's a BDT messagge
        if "START" in str(recv):
            debugStamp("New BDT message START")
            debugStamp(str(recv, 'utf-8'), "Full")
            while True:
                recv = ser.readline()
                debugStamp(str(recv, 'utf-8'), "Full")
                insertDataInDict(recv)
                if "END" in str(recv):
                    debugStamp("New BDT message END")
                    measures["created_at"] = int(time.time())
                    dataToSend.append(measures.copy())
                    stampDataToSend()
                    break
    
    # SEND DATA TO REMOTE SERVER
    # Do it every PERIOD_SERVER seconds and if dataToSend is not empty
    if time.time() - lastSendToServer >= PERIOD_SERVER and dataToSend:
        debugStamp("Sending " + str(len(dataToSend)) + " measures to remote server")
        # Build json to send
        jsonDict["updates"] = dataToSend
        debugStamp(jsonDict, "Full")

        # Send data
        r = requests.post("https://api.thingspeak.com/channels/"+ CHANNEL_ID +"/bulk_update.json", json=jsonDict)
        debugStamp(str(r.status_code) + " " + str(r.reason))

        # Reset data
        # TODO: Capire se fare reset sempre o solo se status_code 202
        # TODO: Capire se salvare le misure non inviate in un file e procedere dopo con l'API che permette di mandare csv
        dataToSend = []
        jsonDict["updates"] = dataToSend

        # Reset timer
        lastSendToServer = time.time()
    