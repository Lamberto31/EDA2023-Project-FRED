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
