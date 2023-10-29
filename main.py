import math
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
USE_HEARTBEAT_PROTOCOL = 0xD9
SOFT_RESTART = 0x40

# Response mode constants.
CONTINUOUS = 0x1
SINGLE = 0x0

# Byte sequence constants.
SCAN_HEADER = (bytes([0xAA]), bytes([0x55]))

port = input("Enter the port: ")
cmd_query = input("Enter the command: ")

ser = serial.Serial(port, 230400, timeout=2, write_timeout=2)
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
    command[1] = USE_HEARTBEAT_PROTOCOL
elif cmd_query == "reset":
    command[1] = SOFT_RESTART
else:
    print("Invalid command given. exiting")
    exit()

ser.write(command)

start_sign = ser.read(2)
response = ser.read(4)
typecode = ser.read()

res_mode = response[3] >> 6

if start_sign == YDLIDAR_START_SIGN:

    print("Retrieved response successfully.")
    res_length = struct.unpack('<4B', response)[0]
    res_type = struct.unpack('<B', typecode)[0]

    print("Response length : ", res_length)
    print("Response type : ", res_type)
    print("Response mode : ", res_mode)

    if res_mode == CONTINUOUS:
        header_count = 0
        total_sample_count = 0

        while True:
            data = ser.read()
            if data == SCAN_HEADER[header_count]:
                header_count = header_count + 1
            if header_count < 2:
                continue

            status = ser.read()
            sample_quantity = ser.read()
            fsa_angle = ser.read(2)
            lsa_angle = ser.read(2)
            checkcode = ser.read(2)

            status = int.from_bytes(status, 'little')
            quantity = int.from_bytes(sample_quantity, 'little')
            fsa = int.from_bytes(fsa_angle, 'little')
            lsa = int.from_bytes(lsa_angle, 'little')

            if status & 0b1 == 1:
                print("Total sample count : ", total_sample_count)
                total_sample_count = 0

            print("New packet received")
            print("Freq : ", ((status & 0b11111110) >> 1) / 10)
            print("Current packet type : ", status & 0b1)
            print("Sample quantity : ", quantity)

            starting_angle = (fsa >> 1) / 64
            ending_angle = (lsa >> 1) / 64
            angle_diff = (ending_angle + 360) - starting_angle if ending_angle - starting_angle < 0 else ending_angle - starting_angle

            print("Starting angle :", starting_angle)
            print("End angle : ", ending_angle)
            print("Checkcode : ", checkcode)

            for i in range(0, quantity):
                sample_data = ser.read(3)
                intensity = sample_data[0]
                intensity = intensity + (sample_data[1] & 0b11) * 256
                distance = (sample_data[2] << 6) + (sample_data[1] >> 2)
                angle = angle_diff / (quantity + 1) * (i + 1) + starting_angle
                correcting_angle = 0 if distance == 0 else math.atan2(21.8 * (155.3 - distance), (155.3 * distance))
                final_angle = math.fmod(angle + correcting_angle, 360)

                print(i + 1, intensity, distance, final_angle)

            total_sample_count = total_sample_count + quantity
            header_count = 0

    elif res_mode == SINGLE:
        payload = ser.read(res_length)
        print("Payload: ", payload)
    else:
        print("Unknown response mode. Prehaps the packet is corrupted?")
else:
    print("Failed to retrieve the response.")