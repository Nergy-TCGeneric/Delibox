import serial
import struct

YDLIDAR_START_SIGN = bytearray([0xA5, 0x5A])
SCANNING = 0x81
HEALTH_STATUS = 0x06
DEVICE_INQUIRY = 0x04

port = input("Enter the port: ")
cmd_query = input("Enter the command: ")

ser = serial.Serial(port, 512000)
command = bytearray([0xA5, 0x00]) 

if cmd_query == "start":
    command[1] = 0x60
elif cmd_query == "stop":
    command[1] = 0x65
elif cmd_query == "dinfo":
    command[1] = 0x90
elif cmd_query == "dhealth":
    command[1] = 0x92
elif cmd_query == "scanfreq":
    command[1] = 0x0D
elif cmd_query == "rangfreq":
    command[1] = 0xD1
elif cmd_query == "pprotect":
    command[1] = 0xD9
elif cmd_query == "reset":
    command[1] = 0x40
else:
    print("Invalid command given. exiting")
    exit()

ser.write(command)

start_sign = ser.read(2)
response = ser.read(4)
typecode = ser.read(1)

if start_sign == YDLIDAR_START_SIGN:
    print("Retrieved response successfully.")
    res_length = struct.unpack('<4B', response)[0]
    res_type = struct.unpack('<B', typecode)[0]

    print("Response length : ", res_length)
    print("Response type : ", res_type)
    
    payload = ser.read(res_length)
    print("Payload: ", payload)
else:
    print("Failed to retrieve the response.")