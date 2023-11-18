import serial
from typing import List
from typing import Tuple 
from dataclasses import dataclass

import math

# Byte sequence constant.
SCAN_HEADER = (bytes([0xAA]), bytes([0x55]))

# Command byte constants.
START_SCAN = bytearray([0xA5, 0x60])
STOP_SCAN = bytearray([0xA5, 0x65])

# Scan data category constants.
CLOUD_DATA = 0
START_DATA = 1

@dataclass
class LaserScanPoint:
    angle: float
    distance: float 

@dataclass
class ScanHeader:
    frequency: float
    packet_type: int
    quantity: int
    start_angle: float
    end_angle: float
    check_code: int


class G2:
    _serial: serial.Serial

    def __init__(self, port):
        self._serial = serial.Serial(port, 230400, timeout=2, write_timeout=2)

    def read_data_once(self, after_iteration=0):
        self._start_scan()
        self._parse_response()
        
        current_iteration: int = 0
        retrieved: List[LaserScanPoint]

        # Wait until it reaches to specified cycle.
        retrieved = self._parse_one_cycle()
        while (current_iteration < after_iteration):
            current_iteration = current_iteration + 1
            retrieved = self._parse_one_cycle()

        self._stop_scan()
        return retrieved

    def _parse_response(self):
        start_sign = self._serial.read(2)
        response = self._serial.read(4)
        typecode = self._serial.read()

    def _start_scan(self):
        self._serial.write(START_SCAN)

    def _stop_scan(self):
        self._serial.write(STOP_SCAN)
    
    def _parse_one_cycle(self) -> List[LaserScanPoint]: 
        header_count: int = 0
        scanned_points: List[LaserScanPoint] = []
        is_first_header: bool = True

        # Loop before the end of a cycle
        while True:
            # detect if this is one of the correct fragment of scan header
            received = self._serial.read()
            if received == SCAN_HEADER[header_count]:
                header_count = header_count + 1
            if header_count < 2:
                continue

            header = self._parse_scan_header_fields()
            parsed_points = self._parse_scan_samples(header)
            scanned_points.extend(parsed_points)

            if not is_first_header and header.packet_type == START_DATA:
                break

            is_first_header = False
            header_count = 0
        
        return scanned_points

    def _parse_scan_header_fields(self) -> ScanHeader:
        # This assumes incoming serial data is aligned correctly.
        # If failed, this does not yield correct results anymore. 
        status = self._serial.read()
        sample_quantity = self._serial.read()
        fsa_angle = self._serial.read(2)
        lsa_angle = self._serial.read(2)
        check_code = self._serial.read(2)

        status = int.from_bytes(status, 'little')
        frequency = (status >> 1) / 10
        packet_type = status & 0b1
        quantity = int.from_bytes(sample_quantity, 'little')
        fsa = int.from_bytes(fsa_angle, 'little')
        lsa = int.from_bytes(lsa_angle, 'little')

        check_code = int.from_bytes(check_code, 'little')

        starting_angle = (fsa >> 1) / 64
        ending_angle = (lsa >> 1) / 64
        angle_diff = (ending_angle + 360) - starting_angle if ending_angle - starting_angle < 0 else ending_angle - starting_angle

        header_field = ScanHeader(frequency, packet_type, quantity, starting_angle, ending_angle, check_code)
        return header_field

    def _parse_scan_samples(self, header: ScanHeader) -> List[LaserScanPoint]:
        # This assumes incoming serial data is aligned correctly.
        # If failed, this does not yield correct results anymore. 
        samples = []
        for i in range(0, header.quantity):
            sample_data = self._serial.read(3)
            sample_data = int.from_bytes(sample_data, 'little')

            second_byte = (sample_data >> 8) & 0b11111111
            third_byte = (sample_data >> 16) & 0b11111111

            angle_diff = (header.end_angle + 360) - header.start_angle if header.end_angle - header.start_angle < 0 else header.end_angle - header.start_angle
            distance = ((third_byte << 6) + (second_byte >> 2))

            angle = angle_diff / (header.quantity + 1) * (i + 1) + header.start_angle
            correcting_angle = 0 if distance == 0 else math.atan2(21.8 * (155.3 - distance), (155.3 * distance))
            final_angle = math.fmod(angle + correcting_angle, 360)
            final_radian = math.radians(final_angle)

            samples.append(LaserScanPoint(final_angle, distance))
        return samples
