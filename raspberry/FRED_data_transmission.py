import serial
import time
# PARAMETER DEFINITION
PERIOD_SERVER = 15 # seconds

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
# Serial connection configuration TODO: capire se è corretta e inserire porta giusta (dopo configurazione bluetooth)
ser = serial.Serial(
    port='/dev/rfcomm1',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

# Check if the serial connection is open
ser.isOpen()

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
                    dataToSend.append(measures)
                    break
    
    # SEND DATA TO REMOTE SERVER
    # TODO: Implementare invio misure via WiFi al server remoto ogni 15 secondi e se ci sono dati