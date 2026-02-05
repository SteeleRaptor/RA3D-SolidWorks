
import smbus
import math

class TemperatureController:
    def __init__(self, root):
        self.root = root

        # I2C Channel to be used
        self.i2cChannel = 1
        # device address (For ADDR connected to GND)
        self.adcAddr = 0x48
        # Register addresses
        self.regConversionAddr = 0x00
        self.regConfigAddr = 0x01
        self.regLoThreshAddr = 0x02
        self.regHiThreshAddr = 0x03
        # Create the bus
        self.bus = smbus.SMBus(self.i2cChannel)

        # Configuration variables
		#    MSB           LSB
		# x.xxx.xxx.x xxx.x.x.x.xx | Config Register division
        # Refer to section 8.1.3 of the datasheet for all of the options
        self.OS        = 0b1   # 0b1   = Start single conversion
        self.MUX       = 0b100 # 0b100 = AIN_P = AIN0 and AIN_N = GND
        self.PGA       = 0b001 # 0b001 = FSR = +/- 4.096V
        self.MODE      = 0b0   # 0b0   = Continuous-conversion mode
        self.DR        = 0b100 # 0b100 = 128 Sa/s
        self.COMP_MODE = 0b0   # 0b0   = Traditional comparator
        self.COMP_POL  = 0b0   # 0b0   = Active low
        self.COMP_LAT  = 0b0   # 0b0   = Nonlatching comparator
        self.COMP_QUE  = 0b11  # 0b11  = Disable comparator

        # Constants for temperature calculation
        self.hotendR = 100000 # Hotend voltage divider resistor value
        self.bedR    = 100000 # Bed voltage divider resistor value
        self.voltageDividerVcc = 3.3
        self.thermistorBeta = 3950 # Beta value for the thermistors (Should be the same for both)
        self.thermistorRo = 100000 # Resistance of thermistors at room temp (Should be the same for both)
        self.thermistorTo = 298.15  # Room temperature value for Ro value in Kelvin (25C)

        self.fullScaleVoltage = 4.096 # This is tied to self.PGA, if one changes, so must the other
        
        # Measurement values
        self.lastADCReading = None
        self.hotendTempCelsius = None
        self.bedTempCelsius = None
        self.measurementState = 0 # 0 = Measure hotend, 1 = Measure bed

        # Set up the config register according to default values stated earlier
        self.setConfigReg()

    def setConfigReg(self):
        # Format the data properly according to data sheet (Section 8.1.3)
        data = [0, 0]
        data[0] = (self.OS << 7) + (self.MUX << 4) + (self.PGA << 1) + self.MODE
        data[1] = (self.DR << 5) + (self.COMP_MODE << 4) + (self.COMP_POL << 3) + (self.COMP_LAT << 2) + self.COMP_QUE
        # Write to the config register
        self.bus.write_i2c_block_data(self.adcAddr, self.regConfigAddr, data)

    def selectADCChannel(self, channel, autoUpdate=True):
        # Change MUX config variable depending on user specified channel
        if channel == 0:
            self.MUX = 0b100 # A0
        elif channel == 1:
            self.MUX = 0b101 # A1
        elif channel == 2:
            self.MUX = 0b110 # A2
        elif channel == 3:
            self.MUX = 0b111 # A3
        else:
            self.MUX = 0b100 # Default to A0 if value is invalid
        # Update the config reg automatically unless user specifies otherwise
        if autoUpdate:
            self.setConfigReg()

    def readADC(self):
        # Read the most recent ADC conversion
        data = self.bus.read_i2c_block_data(self.adcAddr, self.regConversionAddr, 2)
        # Format the reading properly
        rawAdc = (data[0] << 8) + data[1]
        # Make the value negative if needed
        if rawAdc > 32768:
            rawAdc -= 65536
        # Save the last ADC reading
        self.lastADCReading = rawAdc
        # Return the reading
        return self.lastADCReading
    
    def calculateHotendTemp(self, rawAdc):
        # Calculate the measured voltage
        Vmeas = rawAdc / pow(2, 15) * self.fullScaleVoltage
        # Convert the voltage into a resistance
        thermR = Vmeas / (self.voltageDividerVcc - Vmeas) * self.hotendR
        # Calculate the temperature from the resistance
        hotendTempKelvin = 1 / ((1 / self.thermistorTo) + (1 / self.thermistorBeta) * math.log(thermR / self.thermistorRo))
        # Convert to Celsius and save
        self.hotendTempCelsius = hotendTempKelvin - 273.15
        # Return value
        return self.hotendTempCelsius

    def calculateBedTemp(self, rawAdc):
        # Calculate the measured voltage
        Vmeas = rawAdc / pow(2, 15) * self.fullScaleVoltage
        # Convert the voltage into a resistance
        thermR = Vmeas / (self.voltageDividerVcc - Vmeas) * self.bedR
        # Calculate the temperature from the resistance
        bedTempKelvin = 1 / ((1 / self.thermistorTo) + (1 / self.thermistorBeta) * math.log(thermR / self.thermistorRo))
        # Convert to Celsius and save
        self.bedTempCelsius = bedTempKelvin - 273.15
        # Return value
        return self.bedTempCelsius

    def updateTemp(self):
        # Alternates per call which thermistor to measure to not overwhelm the ADC
        if (self.measurementState == 0):
                # Handle reading and calculating hotend temperature
                adcVal = self.readADC()
                self.calculateHotendTemp(adcVal)
                self.root.hotendActual.config(text=f"{round(self.hotendTempCelsius, 2)}°C")
                # Change the ADC channel to A1 (Bed)
                self.selectADCChannel(1)
                self.measurementState = 1 # Toggle measurement state
        else:
                # Handle reading and calculating bed temperature
                adcVal = self.readADC()
                self.calculateBedTemp(adcVal)
                self.root.bedActual.config(text=f"{round(self.bedTempCelsius, 2)}°C")
                # Change the ADC channel to A0 (Hotend)
                self.selectADCChannel(0)
                self.measurementState = 0 # Toggle measurement state