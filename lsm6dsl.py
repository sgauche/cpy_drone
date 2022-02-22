"""
`lsm6dsl`
====================================================

CircuitPython driver for the LSM6DSL 3-axis accelerometer and gyroscope.

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import struct
from time import sleep
from math import radians
from micropython import const

try:
    from typing import Tuple, Optional
except ImportError:
    pass


_CHIP_ID = const(0x6A)

# Register Addresses
_REG_WHO_AM_I     = const(0x0F)
_REG_CTRL1_XL     = const(0x10)
_REG_CTRL2_G      = const(0x11)
_REG_CTRL3_C      = const(0x12)
_REG_CTRL4_C      = const(0x13)
_REG_CTRL6_C      = const(0x15)
_REG_CTRL8_XL     = const(0x17)
#_REG_CTRL9_XL     = const(0x18)
#_REG_CTRL10_C     = const(0x19)
_REG_STATUS       = const(0x1E)
_REG_OUT_TEMP_L   = const(0x20)
_REG_OUTX_L_G     = const(0x22)
_REG_OUTX_L_A     = const(0x28)

class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples: Tuple[str, int, float, Optional[float]]) -> None:
        "creates CV entires"
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value: int) -> bool:
        """Returns true if the given value is a member of the CV"""
        return value in cls.string

class AccelRange(CV):
    """Options for ``accelerometer_range``"""

class GyroRange(CV):
    """Options for ``gyro_data_range``"""

class GyroLPF(CV):
    """Options for ``_gyro_lpf_bw``"""

GyroLPF.add_values(
    (
        ("LPF_NORMAL",      0, 0, None),
        ("LPF_NARROW",      1, 0, None),
        ("LPF_VERY_NARROW", 2, 0, None),
        ("LPF_WIDE",        3, 0, None),
    )
)

class Rate(CV):
    """Options for ``accelerometer_data_rate`` and ``gyro_data_rate``"""

Rate.add_values(
    (
        ("RATE_SHUTDOWN",  0,    0.0, None),
        ("RATE_12_5_HZ",   1,   12.5, None),
        ("RATE_26_HZ",     2,   26.0, None),
        ("RATE_52_HZ",     3,   52.0, None),
        ("RATE_104_HZ",    4,  104.0, None),
        ("RATE_208_HZ",    5,  208.0, None),
        ("RATE_416_HZ",    6,  416.0, None),
        ("RATE_833_HZ",    7,  833.0, None),
        ("RATE_1_66K_HZ",  8, 1660.0, None),
        ("RATE_3_33K_HZ",  9, 3330.0, None),
        ("RATE_6_66K_HZ", 10, 6660.0, None),
        ("RATE_1_6_HZ",   11,    1.6, None),
    )
)


class AccelHPF(CV):
    """Options for the accelerometer high pass filter"""


AccelHPF.add_values(
    (
        ("SLOPE",      0, 0, None),
        ("HPF_DIV100", 1, 0, None),
        ("HPF_DIV9",   2, 0, None),
        ("HPF_DIV400", 3, 0, None),
    )
)

# Other constants
_MILLI_G_TO_ACCEL = 0.00980665


class LSM6DSL:  # pylint: disable=too-many-instance-attributes
    """Base LSM6DSL object. Use :class:`LSM6DSL_I2C` or :class:`LSM6DSL_SPI`
    instead of this. This checks the LSM6DSL was found and enables the sensor for continuous reads

    .. note::
        The operational range of the LSM6DSL is +/-50 gauss.
        Magnetic measurements outside this range may not be as accurate.

    """

    def __init__(self):
        self._accel = None
        self._gyro = None
        self._temp = None
        self._tda = 0
        self._gda = 0
        self._xlda = 0

        self._add_gyro_ranges()
        self._add_accel_ranges()

        self._soft_reset()
        sleep(0.100)
        self._reboot()
        sleep(0.100)

        # CTRL3_C
        # Make sure high and low bytes are set together. Block data update
        self._bdu = True
        # Set to '1' to enable 3-wire SPI.
        self._3wspi = True
        # Register address automatically incremented during a multiple byte access
        self._if_inc = True
        # If ‘1’, an inversion of the low and high parts of the data occurs.
        self._ble = False
        self._write_ctrl3_c()

        # Check device ID.
        chip_id = self._read_byte(_REG_WHO_AM_I)
        if _CHIP_ID != chip_id:
            raise RuntimeError("Failed to find LSM6DSL! Chip ID 0x%x" % chip_id)

        # CTRL1_XL
        # Accelerometer output data rate and power mode selection
        self._odr_xl = Rate.RATE_6_66K_HZ
        # Accelerometer full-scale selection
        self._fs_xl = AccelRange.RANGE_4G
        # Accelerometer digital LPF (LPF1) bandwidth selection.
        self._lpf1_bw_sel = False
        # Accelerometer analog chain bandwidth selection (only for accelerometer
        # ODR ≥ 1.67 kHz)
        self._bw0_xl = False
        self._write_ctrl1_xl()

        # CTRL2_G
        # Gyroscope output data rate selection
        self._odr_g = Rate.RATE_416_HZ
        # Accelerometer full-scale selection
        self._fs_g = GyroRange.RANGE_2000_DPS
        self._write_ctrl2_g()

        # CTRL4_C
        # Disable I2C Interface
        self._i2c_disable = True
        # Enable gyroscope digital LPF1. The bandwidth can be selected through
        # FTYPE[1:0] in CTRL6_C (15h).
        self._gyro_lpf1_en = True
        self._write_ctrl4_c()

        # CTRL6_C
        # Gyroscope's low-pass filter (LPF1) bandwidth selection
        self._gyro_lpf_bw = GyroLPF.LPF_NARROW
        self._write_ctrl6_c()

        # CTRL8_XL
        # Accelerometer low-pass filter LPF2 selection
        self._lpf2_xl_en = True
        # Accelerometer LPF2 and high-pass filter configuration and cutoff setting
        self._hpcf_xl = 3
        # Composite filter input selection.
        self._input_composite = False
        self._write_ctrl8_xl()

        # sleep 30ms to allow measurements to stabilize
        sleep(0.030)

    def _soft_reset(self):
        """Soft reset the sensor. When this bit is set, the configuration registers
        and user registers are reset."""
        self._write_register_byte(_REG_CTRL3_C, 0x01)

    def _reboot(self):
        """Reboot  memory content"""
        self._write_register_byte(_REG_CTRL3_C, 0x80)

    @staticmethod
    def _add_gyro_ranges() -> None:
        GyroRange.add_values(
            (
                ("RANGE_125_DPS", 125,  125,  4.375),
                ("RANGE_250_DPS",   0,  250,  8.75),
                ("RANGE_500_DPS",   1,  500, 17.50),
                ("RANGE_1000_DPS",  2, 1000, 35.0),
                ("RANGE_2000_DPS",  3, 2000, 70.0),
            )
        )

    @staticmethod
    def _add_accel_ranges() -> None:
        AccelRange.add_values(
            (
                ("RANGE_2G",  0,  2, 0.061),
                ("RANGE_16G", 1, 16, 0.488),
                ("RANGE_4G",  2,  4, 0.122),
                ("RANGE_8G",  3,  8, 0.244),
            )
        )

    def _write_ctrl1_xl(self):
        """Write the values to the ctrl1_xl register in the device
        ctrl1_xl sets the accelerometer range and data update rates"""
        self._write_register_byte(_REG_CTRL1_XL, self._ctrl1_xl)

    def _write_ctrl2_g(self):
        """Write the values to the ctrl2_g register in the device"""
        self._write_register_byte(_REG_CTRL2_G, self._ctrl2_g)

    def _write_ctrl3_c(self):
        """Write the values to the ctrl3_c register in the device"""
        self._write_register_byte(_REG_CTRL3_C, self._ctrl3_c)

    def _write_ctrl4_c(self):
        """Write the values to the ctrl4_c register in the device"""
        self._write_register_byte(_REG_CTRL4_C, self._ctrl4_c)

    def _write_ctrl6_c(self):
        """Write the values to the ctrl6_c register in the device"""
        self._write_register_byte(_REG_CTRL6_C, self._ctrl6_c)

    def _write_ctrl8_xl(self):
        """Write the values to the ctrl8_xl register in the device"""
        self._write_register_byte(_REG_CTRL8_XL, self._ctrl8_xl)

    def _read_status(self):
        """Read the value from the status register in the device"""
        return self._read_register(_REG_STATUS, 1)

    def _read_temp(self):
        """Read the value from the out_temp register in the device"""
        return self._read_register(_REG_OUT_TEMP_L, 2)

    def _read_gyro(self):
        """Read the value from the gyro outx, outy, and outz registers in the device"""
        return self._read_register(_REG_OUTX_L_G, 6)

    def _read_accel(self):
        """Read the value from the accel outx, outy, and outz registers in the device"""
        return self._read_register(_REG_OUTX_L_A, 6)

    @property
    def _ctrl1_xl(self):
        """Value to be written to the device's ctrl1_xl register"""
        ctrl1_xl = self._odr_xl << 4
        ctrl1_xl += self._fs_xl << 2
        ctrl1_xl += self._lpf1_bw_sel << 1
        ctrl1_xl += self._bw0_xl
        return ctrl1_xl

    @property
    def _ctrl2_g(self):
        """Value to be written to the device's ctrl2_g register"""
        ctrl2_g = self._odr_g << 4
        ctrl2_g += self._fs_g << 1
        return ctrl2_g

    @property
    def _ctrl3_c(self):
        """Value to be written to the device's ctrl3_c register"""
        ctrl3_c = self._bdu << 6
        ctrl3_c += self._3wspi << 3
        ctrl3_c += self._if_inc << 2
        ctrl3_c += self._ble << 1
        return ctrl3_c

    @property
    def _ctrl4_c(self):
        """Value to be written to the device's ctrl4_c register"""
        ctrl4_c = self._i2c_disable << 1
        ctrl4_c += self._gyro_lpf1_en
        return ctrl4_c

    @property
    def _ctrl6_c(self):
        """Value to be written to the device's ctrl6_c register"""
        ctrl6_c = self._gyro_lpf_bw
        return ctrl6_c

    @property
    def _ctrl8_xl(self):
        """Value to be written to the device's ctrl8_xl register"""
        ctrl8_xl = self._lpf2_xl_en << 7
        ctrl8_xl += self._hpcf_xl << 5
        ctrl8_xl += self._input_composite << 3
        return ctrl8_xl

    @property
    def ble(self):
        """Big little endian: Allowed values are True or False"""
        return self._ble

    @ble.setter
    def ble(self, value):
        if not isinstance(value, bool):
            raise ValueError("Big little endian '%s' not supported" % (value))
        self._ble = value
        self._write_ctrl3_c()

    @property
    def status(self):
        """A tuple representing device
        ``(tda, gda, xlda)``"""
        status_reg = self._read_status
        tda = status_reg & 0b100 > 0
        gda = status_reg & 0b010 > 0
        xlda = status_reg & 0b001
        return (tda, gda, xlda)

    @property
    def temperature(self):
        """The processed temperature sensor value.
        A signed float temperature value in celcius."""
        self._temp = self._read_temp()
        self._temp = list(struct.unpack("<h", bytes(self._temp)))
        return (self._temp[0] / 256.0) + 25.0

    @property
    def gyro(self):
        """The processed gyroscope sensor values.
        A 3-tuple of X, Y, Z axis values in radians/second that are signed floats."""
        self._gyro = self._read_gyro()
        self._gyro = list(struct.unpack("<hhh", bytes(self._gyro)))
        return (
            radians(self._gyro[0] * GyroRange.lsb[self._fs_g] / 1000.0),
            radians(self._gyro[1] * GyroRange.lsb[self._fs_g] / 1000.0),
            radians(self._gyro[2] * GyroRange.lsb[self._fs_g] / 1000.0),
        )

    @property
    def accel(self):
        """The processed accelerometer sensor values.
        A 3-tuple of X, Y, Z axis values in m/s^2 that are signed floats."""
        self._accel = self._read_accel()
        self._accel = list(struct.unpack("<hhh", bytes(self._accel)))
        return (
            self._accel[0] * AccelRange.lsb[self._fs_xl] * _MILLI_G_TO_ACCEL,
            self._accel[1] * AccelRange.lsb[self._fs_xl] * _MILLI_G_TO_ACCEL,
            self._accel[2] * AccelRange.lsb[self._fs_xl] * _MILLI_G_TO_ACCEL,
        )

    ####################### Internal helpers ################################
    def _read_byte(self, register):
        """Read a byte register value and return it"""
        return self._read_register(register, 1)[0]

    def _read_register(self, register, length):
        """Low level register reading, not implemented in base class"""
        raise NotImplementedError()

    def _write_register_byte(self, register, value):
        """Low level register writing, not implemented in base class"""
        raise NotImplementedError()


#class LSM6DSL_I2C(LSM6DSL):
#    """Driver for the LSM6DSL 3-axis magnetometer connected over I2C.
#
#    :param ~busio.I2C i2c: The I2C bus the LSM6DSL is connected to.
#    :param int address: The I2C device address. Defaults to :const:`0x1E`.
#                        but another address can be passed in as an argument
#
#
#    **Quickstart: Importing and using the LSM6DSL**
#
#        Here is an example of using the :class:`LSM6DSL_I2C` class.
#        First you will need to import the libraries to use the sensor
#
#        .. code-block:: python
#
#            import board
#            import LSM6DSL
#
#        Once this is done you can define your `board.I2C` object and define your sensor object
#
#        .. code-block:: python
#
#            i2c = board.I2C()  # uses board.SCL and board.SDA
#            LSM6DSL = LSM6DSL.LSM6DSL_I2C(i2c)
#
#        Now you have access to the :attr:`magnetic` attribute
#
#        .. code-block:: python
#
#            mag_x, mag_y, mag_z = LSM6DSL.magnetic
#
#
#    """
#
#    def __init__(self, i2c, address=0x6A):
#        from adafruit_bus_device import (  # pylint: disable=import-outside-toplevel
#            i2c_device,
#        )
#
#        self._i2c = i2c_device.I2CDevice(i2c, address)
#        super().__init__()
#
#    def _read_register(self, register, length):
#        """Low level register reading over I2C, returns a list of values"""
#        with self._i2c as i2c:
#            i2c.write(bytes([register & 0xFF]))
#            result = bytearray(length)
#            i2c.readinto(result)
#            return result
#
#    def _write_register_byte(self, register, value):
#        """Low level register writing over I2C, writes one 8-bit value"""
#        with self._i2c as i2c:
#            i2c.write(bytes([register & 0xFF, value & 0xFF]))


class LSM6DSL_SPI(LSM6DSL):
    """Driver for SPI connected LSM6DSL.

    :param ~busio.SPI spi: SPI device
    :param ~digitalio.DigitalInOut cs: Chip Select
    :param int baudrate: Clock rate, default is 100000. Can be changed with :meth:`baudrate`


    **Quickstart: Importing and using the LSM6DSL**

        Here is an example of using the :class:`LSM6DSL_SPI` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from digitalio import DigitalInOut, Direction
            import LSM6DSL


        Once this is done you can define your `board.SPI` object and define your sensor object

        .. code-block:: python

            cs = digitalio.DigitalInOut(board.D10)
            spi = board.SPI()
            LSM6DSL = LSM6DSL.LSM6DSL_SPI(spi, cs)

        Now you have access to the :attr:`magnetic` attribute

        .. code-block:: python

            mag_x, mag_y, mag_z = LSM6DSL.magnetic
    """

    def __init__(self, spi, cs, baudrate=100000, polarity=1, phase=0):
        from adafruit_bus_device import (  # pylint: disable=import-outside-toplevel
            spi_device,
        )
        self._spi = spi_device.SPIDevice(spi, cs, baudrate=baudrate, polarity=polarity, phase=phase)
        super().__init__()

    def _read_register(self, register, length):
        """Low level register reading over SPI, returns a list of values"""
        register = (register | 0x80) & 0xFF  # Read single, bit 7 high.
        with self._spi as spi:
            spi.write(bytearray([register]))
            result = bytearray(length)
            spi.readinto(result)
            return result

    def _write_register_byte(self, register, value):
        """Low level register writing over SPI, writes one 8-bit value"""
        register &= 0x7F  # Write, bit 7 low.
        with self._spi as spi:
            spi.write(bytes([register, value & 0xFF]))

