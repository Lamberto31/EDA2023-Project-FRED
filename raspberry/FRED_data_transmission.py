import serial
import time
# INITIAL CONFIGURATION
# Serial connection configuration TODO: capire se è corretta e inserire porta giusta (dopo configurazione bluetooth)
ser = serial.Serial(
    port='/dev/rfcomm0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

# Check if the serial connection is open
ser.isOpen()

# MAIN LOOP: receive data from Bluetooth and send to remote server via WiFi
while True:
    # RECEIVE DATA FROM BLUETOOTH
    # Check if there are incoming data
    if ser.inWaiting() > 0:
        # Read data
        recv = ser.readline()
        # If contains "START" it's a BDT messagge
        if "START" in recv:
            print(str(recv, 'utf-8'))
            while True:
                recv = ser.readline()
                print(str(recv, 'utf-8'))
                if "END" in recv:
                    break
