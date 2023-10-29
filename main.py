import math
import serial
import struct

import matplotlib.pyplot as plt

# Minimum distance constant.
MINIMUM_DISTANCE = 160 # in mm.

# YDLidar protocol category constants.
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

# Scan data category constants.
CLOUD_DATA = 0
START_DATA = 1

# Byte sequence constants.
SCAN_HEADER = (bytes([0xAA]), bytes([0x55]))


angles = []
distances = []
intensities = []

fig = plt.figure()
lidar_polar = plt.subplot(polar=True)
lidar_polar.set_title("G2 Scan result")
lidar_polar.grid(True)

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

            packet_type = status & 0b1
            frequency = ((status & 0b11111110) >> 1) / 10
            starting_angle = (fsa >> 1) / 64
            ending_angle = (lsa >> 1) / 64
            angle_diff = (ending_angle + 360) - starting_angle if ending_angle - starting_angle < 0 else ending_angle - starting_angle

            for i in range(0, quantity):
                sample_data = ser.read(3)
                sample_data = int.from_bytes(sample_data, 'little')

                first_byte = sample_data & 0b11111111
                second_byte = (sample_data >> 8) & 0b11111111
                third_byte = (sample_data >> 16) & 0b11111111

                intensity = first_byte + (second_byte & 0b11) * 256
                distance = ((third_byte << 6) + (second_byte >> 2)) / 1000

                angle = angle_diff / (quantity + 1) * (i + 1) + starting_angle
                correcting_angle = 0 if distance == 0 else math.atan2(21.8 * (155.3 - distance), (155.3 * distance))
                final_angle = math.fmod(angle + correcting_angle, 360)

                angles.append(final_angle)
                distances.append(distance)
                intensities.append(intensity)

            if packet_type == START_DATA:
                print(angles)
                print()
                print(distances)
                lidar_polar.scatter(angles, distances)
                plt.draw()
                plt.pause(0.016)
                lidar_polar.clear()

                angles.clear()
                distances.clear()
                intensities.clear()

            header_count = 0

    elif res_mode == SINGLE:
        payload = ser.read(res_length)
        print("Payload: ", payload)
    else:
        print("Unknown response mode. Prehaps the packet is corrupted?")
else:
    print("Failed to retrieve the response.")

plt.close()