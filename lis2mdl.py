"""
`lis2mdl`
====================================================

CircuitPython driver for the LIS2MDL 3-axis magnetometer.

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import struct
from time import sleep
from micropython import const


_CHIP_ID = const(0x40)

# Register Addresses
_REG_OFFSET_X_L = const(0x45)
_REG_OFFSET_X_H = const(0x46)
_REG_OFFSET_Y_L = const(0x47)
_REG_OFFSET_Y_H = const(0x48)
_REG_OFFSET_Z_L = const(0x49)
_REG_OFFSET_Z_H = const(0x4A)
_REG_WHO_AM_I = const(0x4F)
_REG_CFG_A = const(0x60)
_REG_CFG_B = const(0x61)
_REG_CFG_C = const(0x62)
_REG_INT_CRTL = const(0x63)
_REG_INT_SOURCE = const(0x64)
_REG_INT_THS_L = const(0x65)
_REG_INT_THS_H = const(0x66)
_REG_STATUS = const(0x67)
_REG_OUTX_L = const(0x68)
_REG_OUTX_H = const(0x69)
_REG_OUTY_L = const(0x6A)
_REG_OUTY_H = const(0x6B)
_REG_OUTZ_L = const(0x6C)
_REG_OUTZ_H = const(0x6D)
_REG_TEMP_OUT_L = const(0x6E)
_REG_TEMP_OUT_H = const(0x6F)


class Mode:  # pylint: disable=too-few-public-methods
    """An enum-like class representing the different modes that the LIS2MDL can
    use. The values can be referenced like :attr:`Mode.MODE_CONTINUOUS`
    Possible values are

    - :attr:`Mode.MODE_CONTINUOUS`
    - :attr:`Mode.MODE_SINGLE`
    - :attr:`Mode.MODE_IDLE`

    """

    # pylint: disable=invalid-name
    MODE_CONTINUOUS = const(0x00)
    MODE_SINGLE = const(0x01)
    MODE_IDLE = const(0x03)


_LIS2MDL_MODES = (Mode.MODE_CONTINUOUS, Mode.MODE_SINGLE, Mode.MODE_IDLE)


class Rate:  # pylint: disable=too-few-public-methods
    """An enum-like class representing the different update rates that the LIS2MDL can
    use. The values can be referenced like :attr:`Mode.RATE_10_HZ`
    Possible values are

    - :attr:`Mode.RATE_10_HZ`
    - :attr:`Mode.RATE_20_HZ`
    - :attr:`Mode.RATE_50_HZ`
    - :attr:`Mode.RATE_100_HZ`

    """

    # pylint: disable=invalid-name
    RATE_10_HZ = const(0x00)
    RATE_20_HZ = const(0x01)
    RATE_50_HZ = const(0x02)
    RATE_100_HZ = const(0x03)


_LIS2MDL_DATA_RATES = (
    Rate.RATE_10_HZ,
    Rate.RATE_20_HZ,
    Rate.RATE_50_HZ,
    Rate.RATE_100_HZ,
)

# Other constants
_MAG_SCALE = 0.15  # 1.5 milligauss/LSB * 0.1 microtesla/milligauss


class LIS2MDL:  # pylint: disable=too-many-instance-attributes
    """Base LIS2MDL object. Use :class:`LIS2MDL_I2C` or :class:`LIS2MDL_SPI`
    instead of this. This checks the LIS2MDL was found and enables the sensor for continuous reads

    .. note::
        The operational range of the LIS2MDL is +/-50 gauss.
        Magnetic measurements outside this range may not be as accurate.

    """

    def __init__(self):
        # Check device ID.
        chip_id = self._read_byte(_REG_WHO_AM_I)
        if _CHIP_ID != chip_id:
            raise RuntimeError("Failed to find LIS2MDL! Chip ID 0x%x" % chip_id)

        self._mags = None
        self._temp = None
        #self._offset_x = 0
        #self._offset_y = 0
        #self._offset_z = 0

        self._soft_reset()
        sleep(0.100)
        self._reboot()
        sleep(0.100)

        # CFG_REG_A
        # Enables the magnetometer temperature compensation.
        self._comp_temp_en = True
        # Enables low-power mode.
        self._low_power_mode = False
        # Output data rate configuration.
        self._data_rate = Rate.RATE_100_HZ
        # These bits select the mode of operation of the device.
        self._mode = Mode.MODE_CONTINUOUS
        self._write_cfg_a()

        # CFG_REG_B
        # Enables offset cancellation in single measurement mode
        self._off_canc_one_shot = False
        # Interrupt block checks data after the hard-iron correction
        self._int_on_dataoff = False
        # Selects the frequency of the set pulse.
        self._set_freq = False
        # Enables offset cancellation.
        self._off_canc = True
        # Enables low-pass filter.
        self._lpf = False
        self._write_cfg_b()

        # CFG_REG_C
        # Interrupt signal is driven on the INT/DRDY pin.
        self._int_on_pin = False
        # Inhibit I2C interface. Only the SPI interface can be used.
        self._i2c_dis = True
        # Make sure high and low bytes are set together. Block data update
        self._bdu = True
        # If ‘1’, an inversion of the low and high parts of the data occurs.
        self._ble = False
        # Set to '1' to enable SDO line on pin 7.
        self._4wspi = False
        # If ‘1’, the self-test is enabled.
        self._self_test = False
        # If '1', the data-ready signal is driven on the INT/DRDY pin.
        self._drdy_on_pin = False
        self._write_cfg_c()

        # INT_CTRL_REG
        # Enables the interrupt detection for the X-axis.
        #self._xien = True
        # Enables the interrupt detection for the Y-axis.
        #self._yien = True
        # Enables the interrupt detection for the Z-axis.
        #self._zien = True
        # Controls the polarity of the INT bit when an interrupt occurs.
        #self._int_pol = True
        # Controls whether the INT bit is latched or pulsed.
        #self._int_latch = True
        # Interrupt enable. When set, enables the interrupt generation.
        #self._int_en = True
        #self._write_int_ctrl()

        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        # sleep 30ms to allow measurements to stabilize
        sleep(0.030)

    def _soft_reset(self):
        """Soft reset the sensor. When this bit is set, the configuration registers
        and user registers are reset."""
        self._write_register_byte(_REG_CFG_A, 0x20)

    def _reboot(self):
        """Reboot magnetometer memory content"""
        self._write_register_byte(_REG_CFG_A, 0x40)

    def _write_offset_x_h(self):
        """Write the values to the offset_x_h register in the device"""
        self._write_register_byte(_REG_OFFSET_X_H, self._offset_x_h)

    def _write_offset_x_l(self):
        """Write the values to the offset_x_l register in the device"""
        self._write_register_byte(_REG_OFFSET_X_L, self._offset_x_l)

    def _write_offset_y_h(self):
        """Write the values to the offset_y_h register in the device"""
        self._write_register_byte(_REG_OFFSET_Y_H, self._offset_y_h)

    def _write_offset_y_l(self):
        """Write the values to the offset_y_l register in the device"""
        self._write_register_byte(_REG_OFFSET_Y_L, self._offset_y_l)

    def _write_offset_z_h(self):
        """Write the values to the offset_z_h register in the device"""
        self._write_register_byte(_REG_OFFSET_Z_H, self._offset_z_h)

    def _write_offset_z_l(self):
        """Write the values to the offset_z_l register in the device"""
        self._write_register_byte(_REG_OFFSET_Z_L, self._offset_z_l)

    def _write_cfg_a(self):
        """Write the values to the cfg_a register in the device
        cfg_a sets the low power mode, mode of operation and data update rates"""
        self._write_register_byte(_REG_CFG_A, self._cfg_a)

    def _write_cfg_b(self):
        """Write the values to the cfg_b register in the device"""
        self._write_register_byte(_REG_CFG_B, self._cfg_b)

    def _write_cfg_c(self):
        """Write the values to the cfg_c register in the device"""
        self._write_register_byte(_REG_CFG_C, self._cfg_c)

#    def _write_int_ctrl(self):
#        """Write the values to the int_ctrl register in the device"""
#        self._write_register_byte(_REG_INT_CRTL, self._int_ctrl)

#    def _write_int_ths_l(self):
#        """Write the values to the int_ths_l register in the device"""
#        self._write_register_byte(_REG_INT_THS_L, self._int_ths_l)
#
#    def _write_int_ths_h(self):
#        """Write the values to the int_ths_h register in the device"""
#        self._write_register_byte(_REG_INT_THS_H, self._int_ths_h)

#    def _read_int_source(self):
#        """Read the value from the int_source register in the device"""
#        return self._read_register(_REG_INT_SOURCE, 1)

    def _read_status(self):
        """Read the value from the status register in the device"""
        return self._read_register(_REG_STATUS, 1)

    def _read_mag(self):
        """Read the value from the outx, outy, and outz registers in the device"""
        return self._read_register(_REG_OUTX_L, 6)

    def _read_temp(self):
        """Read the value from the temp_out register in the device"""
        return self._read_register(_REG_TEMP_OUT_L, 2)

    @property
    def _offset_x_l(self):
        """Value to be written to the device's offset_x_l register"""
        return self._offset_x & 0xFF

    @property
    def _offset_x_h(self):
        """Value to be written to the device's offset_x_h register"""
        return (self._offset_x >> 8) & 0xFF

    @property
    def _offset_y_l(self):
        """Value to be written to the device's offset_y_l register"""
        return self._offset_y & 0xFF

    @property
    def _offset_y_h(self):
        """Value to be written to the device's offset_y_h register"""
        return (self._offset_y >> 8) & 0xFF

    @property
    def _offset_z_l(self):
        """Value to be written to the device's offset_z_l register"""
        return self._offset_z & 0xFF

    @property
    def _offset_z_h(self):
        """Value to be written to the device's offset_z_h register"""
        return (self._offset_z >> 8) & 0xFF

    @property
    def _cfg_a(self):
        """Value to be written to the device's cfg_a register"""
        cfg_a = self._comp_temp_en << 7  # True
        cfg_a += self._low_power_mode << 4  # False
        cfg_a += self._data_rate << 2  # 0x00
        cfg_a += self._mode  # 0x00
        return cfg_a

    @property
    def _cfg_b(self):
        """Value to be written to the device's cfg_b register"""
        cfg_b = self._off_canc_one_shot << 4
        cfg_b += self._int_on_dataoff << 3
        cfg_b += self._set_freq << 2
        cfg_b += self._off_canc << 1
        cfg_b += self._lpf
        return cfg_b

    @property
    def _cfg_c(self):
        """Value to be written to the device's cfg_c register"""
        cfg_c = self._int_on_pin << 6
        cfg_c += self._i2c_dis << 5
        cfg_c += self._bdu << 4
        cfg_c += self._ble << 3
        cfg_c += self._4wspi << 2
        cfg_c += self._self_test << 1
        cfg_c += self._drdy_on_pin
        return cfg_c

#    @property
#    def _int_ctrl(self):
#        """Value to be written to the device's int_ctrl register"""
#        int_ctrl = self._xien << 7
#        int_ctrl += self._yien << 6
#        int_ctrl += self._zien << 5
#        int_ctrl += self._int_pol << 2
#        int_ctrl += self._int_latch << 1
#        int_ctrl += self._int_en
#        return int_ctrl

#    @property
#    def _int_ths_l(self):
#        """Value to be written to the device's int_ths_l register"""
#        return self._int_ths & 0xFF
#
#    @property
#    def _int_ths_h(self):
#        """Value to be written to the device's int_ths_h register"""
#        return (self._int_ths >> 8) & 0xFF

    @property
    def offset_x(self):
        """Offset X: Allowed values are between 0 and 436900"""
        return self._offset_x * _MAG_SCALE

    @offset_x.setter
    def offset_x(self, value):
        if int(value / _MAG_SCALE) < -32768 or int(value / _MAG_SCALE) > 32767:
            raise ValueError("Offset X '%s' not supported" % (value))
        self._offset_x = int(value / _MAG_SCALE)
        self._write_offset_x_h()
        self._write_offset_x_l()

    @property
    def offset_y(self):
        """Offset Y: Allowed values are between 0 and 436900"""
        return self._offset_y

    @offset_y.setter
    def offset_y(self, value):
        if value < int(-32768 / _MAG_SCALE) or value > int(32767 / _MAG_SCALE):
            raise ValueError("Offset Y '%s' not supported" % (value))
        self._offset_y = int(value / _MAG_SCALE)
        self._write_offset_y_h()
        self._write_offset_y_l()

    @property
    def offset_z(self):
        """Offset Z: Allowed values are between 0 and 436900"""
        return self._offset_z

    @offset_z.setter
    def offset_z(self, value):
        if value < int(-32768 / _MAG_SCALE) or value > int(32767 / _MAG_SCALE):
            raise ValueError("Offset Z '%s' not supported" % (value))
        self._offset_z = int(value / _MAG_SCALE)
        self._write_offset_z_h()
        self._write_offset_z_l()

    @property
    def mode(self):
        """Operation mode: Allowed values are set in the MODES enum class"""
        return self._mode

    @mode.setter
    def mode(self, value):
        if not value in _LIS2MDL_MODES:
            raise ValueError("Mode '%s' not supported" % (value))
        self._mode = value
        self._write_cfg_a()

    @property
    def data_rate(self):
        """Magnetometer update rate: Allowed values are set in the DATA_RATES enum class"""
        return self._data_rate

    @data_rate.setter
    def data_rate(self, value):
        if not value in _LIS2MDL_DATA_RATES:
            raise ValueError("Data rate '%s' not supported" % (value))
        self._data_rate = value
        self._write_cfg_a()

    @property
    def low_power_mode(self):
        """Low Power Mode: Allowed values are True or False"""
        return self._low_power_mode

    @low_power_mode.setter
    def low_power_mode(self, value):
        if not isinstance(value, bool):
            raise ValueError("Low Power Mode '%s' not supported" % (value))
        self._low_power_mode = value
        self._write_cfg_a()

#    @property
#    def off_cancel_one_shot(self):
#        """Offset cancellation in one shot mode: Allowed values are True or False"""
#        return self._off_canc_one_shot
#
#    @off_cancel_one_shot.setter
#    def off_cancel_one_shot(self, value):
#        if not isinstance(value, bool):
#            raise ValueError(
#                "Offset cancellation in one shot mode '%s' not supported" % (value)
#            )
#        self._off_canc_one_shot = value
#        self._write_cfg_b()

#    @property
#    def int_on_data_off(self):
#        """Interrupt after hard-iron correction: Allowed values are True or False"""
#        return self._int_on_dataoff
#
#    @int_on_data_off.setter
#    def int_on_data_off(self, value):
#        if not isinstance(value, bool):
#            raise ValueError(
#                "Interrupt after hard-iron correction '%s' not supported" % (value)
#            )
#        self._int_on_dataoff = value
#        self._write_cfg_b()

    #    @property
    #    def set_freq(self):
    #        """Frequency of the set pulse: Allowed values are True or False"""
    #        return self._set_freq
    #
    #    @set_freq.setter
    #    def set_freq(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Frequency of the set pulse '%s' not supported" % (value))
    #        self._set_freq = value
    #        self._write_cfg_b()

    @property
    def off_cancel(self):
        """Enables offset cancellation: Allowed values are True or False"""
        return self._off_canc

    @off_cancel.setter
    def off_cancel(self, value):
        if not isinstance(value, bool):
            raise ValueError("Offest cancellation '%s' not supported" % (value))
        self._off_canc = value
        self._write_cfg_b()

    @property
    def lpf(self):
        """Enables low pass filter: Allowed values are True or False"""
        return self._lpf

    @lpf.setter
    def lpf(self, value):
        if not isinstance(value, bool):
            raise ValueError("Low pass filter '%s' not supported" % (value))
        self._lpf = value
        self._write_cfg_b()

    @property
    def int_on_pin(self):
        """IOnterrupt drive signals Allowed values are True or False"""
        return self._int_on_pin

    @int_on_pin.setter
    def int_on_pin(self, value):
        if not isinstance(value, bool):
            raise ValueError("Int on pin '%s' not supported" % (value))
        self._int_on_pin = value
        self._write_cfg_c()

    @property
    def i2c_disable(self):
        """Disable the I2C Bus, only SPI bus can be used: Allowed values are True or False"""
        return self._i2c_dis

    @i2c_disable.setter
    def i2c_disable(self, value):
        if not isinstance(value, bool):
            raise ValueError("Disable I2C Bus '%s' not supported" % (value))
        self._i2c_dis = value
        self._write_cfg_c()

    #    @property
    #    def bdu(self):
    #        """Block data update: Allowed values are True or False"""
    #        return self._bdu
    #
    #    @bdu.setter
    #    def bdu(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Block data update '%s' not supported" % (value))
    #        self._bdu = value
    #        self._write_cfg_c()

    @property
    def ble(self):
        """Big little endian: Allowed values are True or False"""
        return self._ble

    @ble.setter
    def ble(self, value):
        if not isinstance(value, bool):
            raise ValueError("Big little endian '%s' not supported" % (value))
        self._ble = value
        self._write_cfg_c()

#    @property
#    def four_wire_spi(self):
#        """Four wire SPI: Allowed values are True or False"""
#        return self._4wspi
#
#    @four_wire_spi.setter
#    def four_wire_spi(self, value):
#        if not isinstance(value, bool):
#            raise ValueError("Four wire SPI '%s' not supported" % (value))
#        self._4wspi = value
#        self._write_cfg_c()

    #    @property
    #    def self_test(self):
    #        """Self test: Allowed values are True or False"""
    #        return self._self_test
    #
    #    @self_test.setter
    #    def self_test(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Self test '%s' not supported" % (value))
    #        self._self_test = value
    #        self._write_cfg_c()

    #    @property
    #    def drdy_on_pin(self):
    #        """Data ready signal: Allowed values are True or False"""
    #        return self._drdy_on_pin
    #
    #    @drdy_on_pin.setter
    #    def drdy_on_pin(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Data ready signal '%s' not supported" % (value))
    #        self._drdy_on_pin = value
    #        self._write_cfg_c()

    #    @property
    #    def x_int_en(self):
    #        """X Axis Interrupt Enable: Allowed values are True or False"""
    #        return self._xien
    #
    #    @x_int_en.setter
    #    def x_int_en(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("X Axis Interrupt Enable '%s' not supported" % (value))
    #        self._xien = value
    #        self._write_int_ctrl()
    #
    #    @property
    #    def y_int_en(self):
    #        """Y Axis Interrupt Enable: Allowed values are True or False"""
    #        return self._xien
    #
    #    @y_int_en.setter
    #    def y_int_en(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Y Axis Interrupt Enable '%s' not supported" % (value))
    #        self._yien = value
    #        self._write_int_ctrl()
    #
    #    @property
    #    def z_int_en(self):
    #        """Z Axis Interrupt Enable: Allowed values are True or False"""
    #        return self._zien
    #
    #    @z_int_en.setter
    #    def z_int_en(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Z Axis Interrupt Enable '%s' not supported" % (value))
    #        self._zien = value
    #        self._write_int_ctrl()

    #    @property
    #    def int_pol(self):
    #        """Interrupt Polarity: Allowed values are True or False"""
    #        return self._int_pol
    #
    #    @int_pol.setter
    #    def int_pol(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Interrupt Polarity '%s' not supported" % (value))
    #        self._int_pol = value
    #        self._write_int_ctrl()

    #    @property
    #    def int_latch(self):
    #        """Interrupt Latch Enable: Allowed values are True or False"""
    #        return self._int_latch
    #
    #    @int_latch.setter
    #    def int_latch(self, value):
    #        if not isinstance(value, bool):
    #            raise ValueError("Interrupt Latch Enable '%s' not supported" % (value))
    #        self._int_latch = value
    #        self._write_int_ctrl()

    @property
    def int_en(self):
        """Interrupt Enable: Allowed values are True or False"""
        return self._int_en

    @int_en.setter
    def int_en(self, value):
        if not isinstance(value, bool):
            raise ValueError("Interrupt Enable '%s' not supported" % (value))
        self._int_en = value
        self._write_int_ctrl()

#    @property
#    def int_source(self):
#        """A tuple representing interrupts on each axis in a positive and negative direction
#        ``(x_hi, y_hi, z_hi, x_low, y_low, z_low, int_event)``"""
#        int_source = self._read_int_source()
#        x_hi = (int_source & 0b10000000) > 0
#        y_hi = int_source & 0b01000000 > 0
#        z_hi = int_source & 0b00100000 > 0
#
#        x_low = int_source & 0b00010000 > 0
#        y_low = int_source & 0b00001000 > 0
#        z_low = int_source & 0b00000100 > 0
#        int_event = int_source & 0b1 > 0
#        return (x_hi, y_hi, z_hi, x_low, y_low, z_low, int_event)
#
#    @property
#    def int_ths(self):
#        """Interrupt threshold: Allowed values are between 0 and 436900"""
#        return self._int_ths * _MAG_SCALE
#
#    @int_ths.setter
#    def int_ths(self, value):
#        if value < 0 or value > (65535 / _MAG_SCALE):
#            raise ValueError("Interrupt threshold '%s' not supported" % (value))
#        self._int_ths = int(value / _MAG_SCALE)
#        self._write_int_ths_h()
#        self._write_int_ths_l()

    @property
    def status(self):
        """A tuple representing device
        ``(zyxor, zor, yor, xor, zyxda, zda, yda, xda)``"""
        status_reg = self._read_status
        zyxor = status_reg & 0b10000000 > 0
        zor = status_reg & 0b01000000 > 0
        yor = status_reg & 0b00100000 > 0
        xor = status_reg & 0b00010000 > 0
        zyxda = status_reg & 0b00001000 > 0
        zda = status_reg & 0b00000100 > 0
        yda = status_reg & 0b00000010 > 0
        xda = status_reg & 0b1
        return (zyxor, zor, yor, xor, zyxda, zda, yda, xda)

    @property
    def magnetic(self):
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats."""
        self._mags = self._read_mag()
        self._mags = list(struct.unpack("<hhh", bytes(self._mags)))
        return (
            self._mags[0] * _MAG_SCALE,
            self._mags[1] * _MAG_SCALE,
            self._mags[2] * _MAG_SCALE,
        )

    @property
    def temperature(self):
        """The processed temperature sensor value.
        A signed float temperature value in celcius."""
        self._temp = self._read_temp()
        self._temp = list(struct.unpack("<h", bytes(self._temp)))
        return (self._temp[0] / 8.0) + 25.0

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


#class LIS2MDL_I2C(LIS2MDL):
#    """Driver for the LIS2MDL 3-axis magnetometer connected over I2C.
#
#    :param ~busio.I2C i2c: The I2C bus the LIS2MDL is connected to.
#    :param int address: The I2C device address. Defaults to :const:`0x1E`.
#                        but another address can be passed in as an argument
#
#
#    **Quickstart: Importing and using the LIS2MDL**
#
#        Here is an example of using the :class:`LIS2MDL_I2C` class.
#        First you will need to import the libraries to use the sensor
#
#        .. code-block:: python
#
#            import board
#            import lis2mdl
#
#        Once this is done you can define your `board.I2C` object and define your sensor object
#
#        .. code-block:: python
#
#            i2c = board.I2C()  # uses board.SCL and board.SDA
#            lis2mdl = lis2mdl.LIS2MDL_I2C(i2c)
#
#        Now you have access to the :attr:`magnetic` attribute
#
#        .. code-block:: python
#
#            mag_x, mag_y, mag_z = lis2mdl.magnetic
#
#
#    """
#
#    def __init__(self, i2c, address=0x1E):
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


class LIS2MDL_SPI(LIS2MDL):
    """Driver for SPI connected LIS2MDL.

    :param ~busio.SPI spi: SPI device
    :param ~digitalio.DigitalInOut cs: Chip Select
    :param int baudrate: Clock rate, default is 100000. Can be changed with :meth:`baudrate`


    **Quickstart: Importing and using the LIS2MDL**

        Here is an example of using the :class:`LIS2MDL_SPI` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from digitalio import DigitalInOut, Direction
            import lis2mdl


        Once this is done you can define your `board.SPI` object and define your sensor object

        .. code-block:: python

            cs = digitalio.DigitalInOut(board.D10)
            spi = board.SPI()
            lis2mdl = lis2mdl.lis2mdl_SPI(spi, cs)

        Now you have access to the :attr:`magnetic` attribute

        .. code-block:: python

            mag_x, mag_y, mag_z = lis2mdl.magnetic
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
