from periphery import I2C
import time
import atexit

class INA226:
    """
    Python driver for INA226 current, voltage, and power monitor using periphery.I2C.
    Supports shunt resistor R_SHUNT = 0.01 Ohm (R010).
    Based on INA226 datasheet and example calculations.

    Register addresses (from datasheet):
    - CONFIG: 0x00
    - SHUNT_VOLTAGE: 0x01
    - BUS_VOLTAGE: 0x02
    - POWER: 0x03
    - CURRENT: 0x04
    - CALIBRATION: 0x05
    - MASK_ENABLE: 0x06 (optional for alerts)

    Calibration for R_SHUNT=0.01 Ohm:
    - Max expected current: 0.08192 / 0.01 = 8.192A
    - CURRENT_LSB = 0.08192 / 0.01 / 2^15 = 0.00025A = 0.25mA
    - CAL = 0.00512 / (R_SHUNT * CURRENT_LSB) ≈ 2097
    - POWER_LSB = 20 * CURRENT_LSB ≈ 0.0048828 W
    """

    # Register addresses
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05
    REG_MASK_ENABLE = 0x06

    # Default configuration: Continuous shunt and bus voltage measurements
    # Averaging: 16 samples, Conversion time: 1.1ms for both shunt and bus
    CONFIG_DEFAULT = 0x4127  # (16 avg) | (shunt 1.1ms) | (bus 1.1ms) | continuous mode

    # LSB values (for R_SHUNT=0.01 Ohm, max 8.192A)
    CURRENT_LSB = 0.00025          # A
    # POWER_LSB = 0.0048828          # W
    SHUNT_VOLTAGE_LSB = 0.0000025  # 2.5 uV
    BUS_VOLTAGE_LSB = 0.00125      # 1.25 mV
    CALIBRATION_VALUE = 2048       # int(0.00512 / (0.01 * CURRENT_LSB))
    

    def __init__(self, dev="/dev/i2c-4", address=0x40):
        """
        Initialize INA226.

        :param i2c: periphery.I2C instance (e.g., I2C("/dev/i2c-1"))
        :param address: I2C address of INA226 (default 0x40)
        """
        self.i2c = I2C(dev)
        self.address = address
        self._configured = False
        atexit.register(self.i2c.close)

    def _read_word(self, register):
        """Read 16-bit word from register (handles signed values)."""
        msg = [
            I2C.Message([register], read=False),  # Write register address
            I2C.Message([0x00, 0x00], read=True)  # Read 2 bytes
        ]
        self.i2c.transfer(self.address, msg)
        msb, lsb = msg[1].data
        value = (msb << 8) | lsb
        # Convert to signed if MSB is set (for negative values like shunt voltage)
        if value & 0x8000:
            value -= 0x10000
        return value

    def _write_word(self, register, value):
        """Write 16-bit word to register."""
        msb = (value >> 8) & 0xFF
        lsb = value & 0xFF
        msg = [I2C.Message([register, msb, lsb], read=False)]
        self.i2c.transfer(self.address, msg)

    def configure(self, config=CONFIG_DEFAULT):
        """
        Configure the INA226.

        :param config: Configuration register value (default: continuous mode, 16 avg, 1.1ms conv time)
        """
        self._write_word(self.REG_CONFIG, config)
        self._write_word(self.REG_CALIBRATION, self.CALIBRATION_VALUE)
        self._configured = True
        time.sleep(0.01)  # Allow settling

    def read_shunt_voltage(self):
        """Read shunt voltage in mV (signed)."""
        if not self._configured:
            self.configure()
        raw = self._read_word(self.REG_SHUNT_VOLTAGE)
        return raw * self.SHUNT_VOLTAGE_LSB * 1000  # Convert to mV

    def read_bus_voltage(self):
        """Read bus voltage in V."""
        if not self._configured:
            self.configure()
        raw = self._read_word(self.REG_BUS_VOLTAGE)
        return raw * self.BUS_VOLTAGE_LSB

    def read_current(self):
        """Read current in A (signed)."""
        if not self._configured:
            self.configure()
        raw = self._read_word(self.REG_CURRENT)
        return raw * self.CURRENT_LSB

    @property
    def shunt_voltage(self):
        """Shunt voltage in mV (property for easy access)."""
        return self.read_shunt_voltage()

    @property
    def bus_voltage(self):
        """Bus voltage in V."""
        return self.read_bus_voltage()

    @property
    def current(self):
        """Current in A."""
        return self.read_current()

    def reset(self):
        """Reset the INA226."""
        config = self.CONFIG_DEFAULT | 0x8000  # Set RST bit
        self._write_word(self.REG_CONFIG, config)
        time.sleep(0.01)
        self.configure()

# Example usage and test
if __name__ == '__main__':
    from periphery import I2C

    # Initialize I2C bus (adjust for Radxa Zero 3W, typically /dev/i2c-1)
    # Create INA226 instance (address 0x40 is default)
    ina = INA226(address=0x40)

    try:
        # Configure and calibrate
        ina.configure()
        print("INA226 configured with R_SHUNT=0.01 Ohm, max current ~8A")

        # Continuous reading loop
        while True:
            print(f"Bus Voltage: {ina.bus_voltage:.3f} V")
            print(f"Shunt Voltage: {ina.shunt_voltage:.3f} mV")
            print(f"Current: {ina.current:.3f} A")
            # print(f"Power: {ina.power:.3f} W")
            # print(f"Load Voltage: {ina.get_load_voltage():.3f} V")
            # print("-" * 40)
            time.sleep(1)

    except Exception as e:
        print(f"Error: {e}")