import serial
import time

# serial port settings
port = "COM13"
baudrate = 9600
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE

# open the serial port
ser = serial.Serial(port, baudrate, bytesize, parity, stopbits)
print("Serial port", port, "opened")
# Send some data

while True:
    ser.write(bytes("TEST\n", 'utf-8'))
    print("Data Sent")
    # Wait a bit
    print("Waiting for data")
    time.sleep(1)
    # Read all the data available
    while ser.in_waiting > 0:
        serial_data = ser.readline().decode().strip()
        print("Data Received: ", serial_data)
