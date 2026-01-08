"""
BNO055 IMU Sensor Interface

Handles communication with the BNO055 via I2C and provides
orientation data using quaternions (no gimbal lock) plus
Euler angles for display.
"""

import time
import math
from smbus2 import SMBus
from dataclasses import dataclass


@dataclass
class Quaternion:
    """Unit quaternion for rotation (w, x, y, z)."""
    w: float
    x: float
    y: float
    z: float

    def to_rotation_matrix(self) -> list:
        """
        Convert quaternion to 4x4 rotation matrix (OpenGL column-major).
        Returns a flat list of 16 floats for glMultMatrixf().
        """
        w, x, y, z = self.w, self.x, self.y, self.z

        # Rotation matrix from quaternion
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z

        # Column-major order for OpenGL
        return [
            1 - 2 * (yy + zz),  2 * (xy + wz),      2 * (xz - wy),      0,
            2 * (xy - wz),      1 - 2 * (xx + zz),  2 * (yz + wx),      0,
            2 * (xz + wy),      2 * (yz - wx),      1 - 2 * (xx + yy),  0,
            0,                  0,                  0,                  1
        ]

    def to_euler(self) -> tuple:
        """
        Convert quaternion to Euler angles (heading, roll, pitch) in degrees.
        Used for display only - not for rotation.

        In our OpenGL frame:
        - X = forward (roll axis)
        - Y = up (heading/yaw axis)
        - Z = right (pitch axis)
        """
        w, x, y, z = self.w, self.x, self.y, self.z

        # Heading/Yaw (rotation around Y - up axis)
        siny_cosp = 2 * (w * y + x * z)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        heading = math.atan2(siny_cosp, cosy_cosp)

        # Pitch (rotation around Z - right axis, nose up/down)
        sinp = 2 * (w * z - x * y)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Roll (rotation around X - forward axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + z * z)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Convert to degrees, adjust heading to 0-360
        heading_deg = math.degrees(heading)
        if heading_deg < 0:
            heading_deg += 360

        return (heading_deg, math.degrees(roll), math.degrees(pitch))


@dataclass
class Orientation:
    """Orientation with quaternion (for rotation) and Euler angles (for display)."""
    quaternion: Quaternion
    heading: float  # 0-360, compass direction (from quaternion)
    roll: float     # rotation around forward axis (from quaternion)
    pitch: float    # nose up/down (from quaternion)

    @classmethod
    def from_quaternion(cls, quat: Quaternion) -> 'Orientation':
        """Create Orientation from quaternion, computing Euler angles."""
        h, r, p = quat.to_euler()
        return cls(quaternion=quat, heading=h, roll=r, pitch=p)


class BNO055:
    """BNO055 9-DOF IMU sensor interface."""

    # Register addresses
    OPR_MODE = 0x3D
    PWR_MODE = 0x3E
    SYS_TRIGGER = 0x3F
    UNIT_SEL = 0x3B
    EULER_H_LSB = 0x1A
    QUAT_W_LSB = 0x20  # Quaternion data: W, X, Y, Z (8 bytes, 1/16384 units)
    CALIB_STAT = 0x35

    # Operating modes
    MODE_CONFIG = 0x00
    MODE_NDOF = 0x0C

    def __init__(self, bus_num: int = 2, address: int = 0x29):
        """
        Initialize BNO055 sensor.

        Args:
            bus_num: I2C bus number (2 for i915 gmbus vga on X200)
            address: I2C address (0x29 if ADR pin high, 0x28 if low)
        """
        self.bus_num = bus_num
        self.address = address
        self.bus = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()

    def open(self):
        """Open I2C bus and initialize sensor."""
        self.bus = SMBus(self.bus_num)
        self._init_sensor()

    def close(self):
        """Close I2C bus."""
        if self.bus:
            self.bus.close()
            self.bus = None

    def _write(self, reg: int, val: int):
        """Write byte to register."""
        self.bus.write_byte_data(self.address, reg, val)

    def _read(self, reg: int, length: int) -> list:
        """Read bytes from register."""
        return self.bus.read_i2c_block_data(self.address, reg, length)

    def _init_sensor(self):
        """Reset and configure sensor for NDOF mode."""
        # Enter config mode
        self._write(self.OPR_MODE, self.MODE_CONFIG)
        time.sleep(0.025)

        # Reset
        self._write(self.SYS_TRIGGER, 0x20)
        time.sleep(0.65)

        # Normal power mode
        self._write(self.PWR_MODE, 0x00)
        time.sleep(0.01)

        # Units: degrees, Celsius, m/s^2
        self._write(self.UNIT_SEL, 0x00)
        time.sleep(0.01)

        # NDOF mode (full sensor fusion)
        self._write(self.OPR_MODE, self.MODE_NDOF)
        time.sleep(0.02)

    def _convert_signed16(self, val: int) -> int:
        """Convert 16-bit unsigned to signed."""
        if val > 32767:
            val -= 65536
        return val

    def read_euler(self) -> Orientation:
        """
        Read quaternion from sensor and return Orientation.

        Uses quaternion internally to avoid gimbal lock, but provides
        Euler angles (heading, roll, pitch) for display purposes.

        Returns:
            Orientation with quaternion and derived Euler angles
        """
        # Read quaternion data (W, X, Y, Z - each 16-bit, 1/16384 scale)
        data = self._read(self.QUAT_W_LSB, 8)

        w = (data[1] << 8) | data[0]
        x = (data[3] << 8) | data[2]
        y = (data[5] << 8) | data[4]
        z = (data[7] << 8) | data[6]

        # Convert to signed and normalize (1/2^14 = 1/16384 units)
        scale = 1.0 / 16384.0
        bno_w = self._convert_signed16(w) * scale
        bno_x = self._convert_signed16(x) * scale
        bno_y = self._convert_signed16(y) * scale
        bno_z = self._convert_signed16(z) * scale

        # Remap from BNO055 frame to OpenGL visualization frame:
        # BNO055: X=right, Y=forward, Z=up
        # OpenGL: X=forward, Y=up, Z=right
        # Quaternion components map to rotation axes, so remap accordingly
        quat = Quaternion(
            w=bno_w,
            x=bno_y,   # OpenGL X (forward) = BNO Y (forward)
            y=bno_z,   # OpenGL Y (up) = BNO Z (up)
            z=bno_x    # OpenGL Z (right) = BNO X (right)
        )

        return Orientation.from_quaternion(quat)

    def get_calibration(self) -> tuple:
        """
        Get calibration status.

        Returns:
            Tuple of (sys, gyro, accel, mag) calibration levels (0-3)
        """
        data = self._read(self.CALIB_STAT, 1)[0]
        return (
            (data >> 6) & 0x03,  # sys
            (data >> 4) & 0x03,  # gyro
            (data >> 2) & 0x03,  # accel
            data & 0x03          # mag
        )


class MockSensor:
    """
    Mock sensor for testing without hardware.
    Generates smoothly varying orientation using quaternions.
    """

    def __init__(self):
        self.t = 0

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read_euler(self) -> Orientation:
        """Generate fake orientation data using quaternions."""
        self.t += 0.02

        # Generate rotation from Euler angles, then convert to quaternion
        # Our frame: X=forward (roll), Y=up (heading), Z=right (pitch)
        heading = math.radians((self.t * 20) % 360)  # around Y
        roll = math.radians(30 * math.sin(self.t * 0.5))  # around X
        pitch = math.radians(20 * math.sin(self.t * 0.3))  # around Z

        # Euler to quaternion: apply heading (Y), then pitch (Z), then roll (X)
        ch, sh = math.cos(heading / 2), math.sin(heading / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)

        # Combined quaternion for Y-Z-X rotation order
        quat = Quaternion(
            w=cr * cp * ch + sr * sp * sh,
            x=sr * cp * ch - cr * sp * sh,
            y=cr * cp * sh + sr * sp * ch,
            z=cr * sp * ch - sr * cp * sh
        )

        return Orientation.from_quaternion(quat)

    def get_calibration(self) -> tuple:
        return (3, 3, 3, 3)
