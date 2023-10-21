import serial
import struct

print("Enter the port: ")
port = input()
ser = serial.Serial(port, 512000)

print("Enter the command: ")
cmd_query = input()

command = bytearray() 
if cmd_query == "start":
    command = bytearray([0xA5, 0x60])
elif cmd_query == "stop":
    command = bytearray([0xA5, 0x65])
elif cmd_query == "dinfo":
    command = bytearray([0xA5, 0x90])
elif cmd_query == "dhealth":
    command = bytearray([0xA5, 0x92])

ser.write(command)

start_sign = ser.read(2)
response = ser.read(4)
typecode = ser.read(1)

if start_sign == bytearray([0xA5, 0x5A]):
    print("Retrieved response successfully.")
    print("Response length : ", struct.unpack('<4B', response)[0])
    print("Response type : ", struct.unpack('<B', typecode)[0])
    
    payload = ser.read(20)
    print("Payload: ", payload)
else:
    print("Failed to retrieve the response.")