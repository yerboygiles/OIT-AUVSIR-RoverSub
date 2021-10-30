import serial

serial = serial.Serial('/dev/ttyAMA0', 115200)
serial.flushInput()

while 1:
    print(serial.readline())
