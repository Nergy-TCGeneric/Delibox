import smbus
import time

MPU9250_WHO_AM_I = 0x75

# MPU9250 registers.
# Accelerometer.
MPU9250_ACCEL_XOUT_H = 0x3B
MPU9250_ACCEL_YOUT_H = 0x3D
MPU9250_ACCEL_ZOUT_H = 0x3F

# Gyroscope.
MPU9250_GYRO_XOUT_H = 0x43
MPU9250_GYRO_YOUT_H = 0x45
MPU9250_GYRO_ZOUT_H = 0x47

# Magnetometer.
MPU9250_RA_XOUT_L = 0x03
MPU9250_RA_XOUT_H = 0x04
MPU9250_RA_YOUT_L = 0x05
MPU9250_RA_YOUT_H = 0x06
MPU9250_RA_ZOUT_L = 0x07
MPU9250_RA_ZOUT_H = 0x08

# Configuration.
MPU9250_CONFIG = 0x1A
MPU9250_ACC_DLPF = 0x1D
MPU9250_GYRO_CONFIG = 0x1B
MPU9250_ACC_CONFIG_1 = 0x1C
MPU9250_ACC_CONFIG_2 = 0x1D
MPU9250_POWER_MGNT = 0x6B
MPU9250_USER_CTRL = 0x6A
AK8963_CTRL_1 = 0x0A
AK8963_ST2 = 0x09
AK8963_ASA_X = 0x10
AK8963_ASA_Y = 0x11
AK8963_ASA_Z = 0x12

# Scale factors.
# https://github.com/bolderflight/invensense-imu/blob/main/src/mpu9250.cpp#L165
ACCEL_SCALE_FACTOR = 8.0 / 32767.5  # Accelerator range +- 8g
GYRO_SCALE_FACTOR = 25.0 / 32767.5  # Gyro degree per second +- 250
MAGNETIC_SCALE = 4912 / 32760  # Assuming 16-bit output. unit is uT.


class MPU9250:
    address: int
    bus: smbus.SMBus
    accel_data: list[int]
    gyro_data: list[int]
    compass_data: list[int]
    compass_adjustment: list[int]

    def __init__(self, address=0x68, bus_number=1):
        self.MPU9250_ADDRESS = address
        self.bus = smbus.SMBus(bus_number)

        self.accel_data = [0, 0, 0]
        self.gyro_data = [0, 0, 0]
        self.compass_data = [0, 0, 0]

        # This needs some time to avoid Remote IOError.
        # https://stackoverflow.com/questions/52735862/getting-ioerror-errno-121-remote-i-o-error-with-smbus-on-python-raspberry-w
        time.sleep(1)
        self._init()

        who_am_i = self.read_byte(MPU9250_WHO_AM_I)
        if who_am_i == 0x71:
            print("MPU_9250 is connected.")
        else:
            print("MPU_9250 connection failed. WHO_AM_I =", who_am_i)
            exit()

    def _init(self):
        # https://github.com/bolderflight/invensense-imu/blob/main/src/mpu9250.cpp#L46

        # Power down AK8963.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, AK8963_CTRL_1, 0x00)
        # Wait just a bit longer, transition might take a while
        time.sleep(0.1)
        # Access to Fuse ROM to retrieve adjustment values.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, AK8963_CTRL_1, 0x0F)
        # Wait for it
        time.sleep(0.1)
        # Magnetometer adjustment values for X, Y and Z.
        self.compass_adjustment[0] = (
            (self.read_byte(AK8963_ASA_X) - 128.0) / 256.0 + 1.0
        ) * MAGNETIC_SCALE
        self.compass_adjustment[1] = (
            (self.read_byte(AK8963_ASA_Y) - 128.0) / 256.0 + 1.0
        ) * MAGNETIC_SCALE
        self.compass_adjustment[2] = (
            (self.read_byte(AK8963_ASA_Z) - 128.0) / 256.0 + 1.0
        ) * MAGNETIC_SCALE
        # Power down AK8963.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, AK8963_CTRL_1, 0x00)
        # Let AK8963 emit the 16-bit output and update in 100Hz frequency.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, AK8963_CTRL_1, 0x16)

        # Reset the MPU9250.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, MPU9250_POWER_MGNT, 0x80)
        # Wait just a bit until MPU9250 gets back to normal
        time.sleep(0.001)
        # Select the best available clock source - PLL or 20MHz internal osciliator.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, MPU9250_POWER_MGNT, 0x01)
        # Enable I2C master mode.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x20)
        # Set Accelerometer range to +- 8g.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, MPU9250_ACC_CONFIG_1, 0x10)
        # Set Gyroscope range to +- 250 dps.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00)
        # Set accelerometer's DLPF BW to 20Hz. This will induce about 8.87ms delay to output signal.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, MPU9250_ACC_CONFIG_2, 0x04)

    def read_byte(self, reg):
        return self.bus.read_byte_data(self.MPU9250_ADDRESS, reg)

    def read_word(self, reg, reverse=False):
        offset = -1 if reverse else 1
        high = self.bus.read_byte_data(self.MPU9250_ADDRESS, reg)
        low = self.bus.read_byte_data(self.MPU9250_ADDRESS, reg + offset)
        value = (high << 8) + low
        return value

    def read_acceleration(self):
        x = self.read_word(MPU9250_ACCEL_XOUT_H)
        y = self.read_word(MPU9250_ACCEL_YOUT_H)
        z = self.read_word(MPU9250_ACCEL_ZOUT_H)
        self.accel_data = [
            x / ACCEL_SCALE_FACTOR,
            y / ACCEL_SCALE_FACTOR,
            z / ACCEL_SCALE_FACTOR,
        ]

    def read_gyroscope(self):
        x = self.read_word(MPU9250_GYRO_XOUT_H)
        y = self.read_word(MPU9250_GYRO_YOUT_H)
        z = self.read_word(MPU9250_GYRO_ZOUT_H)
        self.gyro_data = [
            x * GYRO_SCALE_FACTOR,
            y * GYRO_SCALE_FACTOR,
            z * GYRO_SCALE_FACTOR,
        ]

    def read_ra(self):
        x = self.read_word(MPU9250_RA_XOUT_H, True)
        y = self.read_word(MPU9250_RA_YOUT_H, True)
        z = self.read_word(MPU9250_RA_ZOUT_H, True)
        self.compass_data = [
            x * self.compass_adjustment[0],
            y * self.compass_adjustment[1],
            z * self.compass_adjustment[2],
        ]

        # We must read ST2 register in order to update the magnetic measurement
        # https://download.mikroe.com/documents/datasheets/ak8963c-datasheet.pdf
        self.read_word(AK8963_ST2)

    def update_data(self):
        self.read_acceleration()
        self.read_gyroscope()
        self.read_ra()

    def get_acceleration(self):
        return self.accel_data

    def get_gyroscope(self):
        return self.gyro_data

    def get_ra(self):
        return self.compass_data