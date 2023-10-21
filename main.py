import serial
import struct

YDLIDAR_START_SIGN = bytearray([0xA5, 0x5A])
SCANNING = 0x81
HEALTH_STATUS = 0x06
DEVICE_INQUIRY = 0x04

# Byte constants.
START_SCAN = 0x60
STOP_SCAN = 0x65
GET_DEVICE_INFO = 0x90
GET_HEALTH_STATUS = 0x92
INCREASE_SCAN_FREQ = 0x09
DECREASE_SCAN_FREQ = 0x0A
INCREASE_SCAN_FREQ_10X = 0x0B
DECREASE_SCAN_FREQ_10X = 0x0C
GET_CURRENT_SCAN_FREQ = 0x0D
GET_CURRENT_RANGE_FREQ = 0xD1
POWER_DOWN_PROTECT = 0xD9
SOFT_RESTART = 0x40

port = input("Enter the port: ")
cmd_query = input("Enter the command: ")

ser = serial.Serial(port, 512000)
command = bytearray([0xA5, 0x00]) 

if cmd_query == "start":
    command[1] = START_SCAN 
elif cmd_query == "stop":
    command[1] = STOP_SCAN
elif cmd_query == "dinfo":
    command[1] = GET_DEVICE_INFO
elif cmd_query == "dhealth":
    command[1] = GET_HEALTH_STATUS 
elif cmd_query == "scanfreq":
    command[1] = GET_CURRENT_SCAN_FREQ
elif cmd_query == "rangfreq":
    command[1] = GET_CURRENT_RANGE_FREQ
elif cmd_query == "pprotect":
    command[1] = POWER_DOWN_PROTECT
elif cmd_query == "reset":
    command[1] = SOFT_RESTART
else:
    print("Invalid command given. exiting")
    exit()

ser.write(command)

start_sign = ser.read(2)
response = ser.read(4)
res_mode = response[3] >> 6
typecode = ser.read(1)

if start_sign == YDLIDAR_START_SIGN:
    print("Retrieved response successfully.")
    res_length = struct.unpack('<4B', response)[0]
    res_type = struct.unpack('<B', typecode)[0]

    print("Response length : ", res_length)
    print("Response type : ", res_type)
    print("Response mode : ", res_mode)
    
    payload = ser.read(res_length)
    print("Payload: ", payload)
else:
    print("Failed to retrieve the response.")