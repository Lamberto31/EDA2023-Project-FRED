import serial
import time
import requests
from decouple import config

# PARAMETER DEFINITION
PERIOD_SERVER = 15 # seconds

# GET SECRET VALUE
API_KEY = config('API_KEY')
CHANNEL_ID = config('CHANNEL_ID')

# FUNCTION DEFINITION
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


# INITIAL CONFIGURATION
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
    print("Serial connection started")
else:
    print("Serial connection not started, try again")
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
dataToSend =  []

# Init json to send
jsonDict = {}
jsonDict["write_api_key"] = API_KEY
jsonDict["updates"] = dataToSend

# Init last execution time
lastSendToServer = time.time()


# MAIN LOOP: receive data from Bluetooth and send to remote server via WiFi
while True:
    # RECEIVE DATA FROM BLUETOOTH
    # Check if there are incoming data
    if ser.inWaiting() > 0:
        # Read data
        recv = ser.readline()
        # If contains "START" it's a BDT messagge
        if "START" in str(recv):
            print(str(recv, 'utf-8'))
            while True:
                recv = ser.readline()
                print(str(recv, 'utf-8'))
                insertDataInDict(recv)
                if "END" in str(recv):
                    measures["created_at"] = int(time.time())
                    dataToSend.append(measures.copy())
                    break
    
    # SEND DATA TO REMOTE SERVER
    # Do it every PERIOD_SERVER seconds and if dataToSend is not empty
    if time.time() - lastSendToServer >= PERIOD_SERVER and dataToSend:
        # Build json to send
        jsonDict["updates"] = dataToSend
        print(jsonDict)

        # Send data
        r = requests.post("https://api.thingspeak.com/channels/"+ CHANNEL_ID +"/bulk_update.json", json=jsonDict)
        print(r.status_code, r.reason)

        # Reset data
        # TODO: Capire se fare reset sempre o solo se status_code 202
        dataToSend = []
        jsonDict["updates"] = dataToSend

        # Reset timer
        lastSendToServer = time.time()
    