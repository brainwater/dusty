import board
import digitalio
import pwmio
import time
import busio
import adafruit_scd4x
import adafruit_bmp180
import ipaddress
import ssl
import wifi
import socketpool
import adafruit_requests
from micropython import const
from adafruit_pm25.uart import PM25_UART
from adafruit_io.adafruit_io import IO_HTTP

try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise
# A0 and MI are PWM
# A1 and MO are tachometer


class Color:
    RED = (1, 0, 0)
    GREEN = (0, 1, 0)
    BLUE = (0, 0, 1)

    YELLOW = (1, 0.4, 0)


class Business:
    reset_pin = None

    def __init__(self):
        self.fanspwm = tuple([pwmio.PWMOut(pin, frequency=25*(10**3)) for pin in [board.MISO, board.A0]])
        for i in range(90):
            time.sleep(0.05)
            self.setFans(i / 100.0)
        i2c = board.I2C()
        self.scd4x = adafruit_scd4x.SCD4X(i2c)
        print("SCD4X Serial number:", [hex(i) for i in self.scd4x.serial_number])
        self.scd4x.start_periodic_measurement()

        self.bmp180 = adafruit_bmp180.Adafruit_BMP180_I2C(i2c)
        self.bmp180.sea_level_pressure = 1013.25 # TODO: Can I use this for updated local weather from the internet? or what?

        self.rgbLed = tuple([pwmio.PWMOut(led) for led in [board.SCK, board.A3, board.A2]])
        for led in self.rgbLed:
            led.duty_cycle = 2**12-1
        time.sleep(0.4)
        uart = busio.UART(board.TX, board.RX, baudrate=9600)
        self.pm25 = PM25_UART(uart, self.reset_pin)
        print("Found PM2.5 sensor, reading data...")

        

        print("Available WiFi networks:")
        for network in wifi.radio.start_scanning_networks():
            print("\t%s\t\tRSSI: %d\tChannel: %d" % (str(network.ssid, "utf-8"),
                    network.rssi, network.channel))
        wifi.radio.stop_scanning_networks()
        print("Connecting to %s"%secrets["ssid"])
        # TODO: Add retry limit and graceful failure of missing network
        found = False
        while not found:
            try:
                wifi.radio.connect(secrets["ssid"], secrets["password"])
                print("Connected to %s!"%secrets["ssid"])
                found = True
            except ConnectionError:
                print("Retrying")
                time.sleep(5)
        pool = socketpool.SocketPool(wifi.radio)
        requests = adafruit_requests.Session(pool, ssl.create_default_context())

        self.aio = IO_HTTP(secrets["ADAFRUIT_IO_USERNAME"], secrets["ADAFRUIT_IO_KEY"], requests)

        self.temperature_feed_bmp = self.aio.get_feed('dusty.temperature-bmp180')
        self.temperature_feed_scd = self.aio.get_feed('dusty.temperature-scd41')
        self.pressure_feed = self.aio.get_feed('dusty.pressure')
        self.co2_feed = self.aio.get_feed('dusty.co2')
        self.humidity_feed = self.aio.get_feed('dusty.humidity')
        self.pm25_feed = self.aio.get_feed('dusty.pm25')
        self.pm10_feed = self.aio.get_feed('dusty.pm10')

    def setFanPWM(self, percent, pin):
        if percent > 1 or percent < 0:
            print("ERROR! percent must be between 0 and 1")
            return
        pin.duty_cycle = int(percent*(2**16-1))

    def setRGB(self, rgbValue, percent=0.1):
        for led, val in zip(self.rgbLed, rgbValue):
            led.duty_cycle = int((2**16-1) * percent * val)

    def printaqdata(self, aqdata):
        print()
        print("Concentration Units (standard)")
        print("---------------------------------------")
        print(
            "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
            % (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"])
        )
        print("Concentration Units (environmental)")
        print("---------------------------------------")
        print(
            "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
                % (aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"])
        )
        print("---------------------------------------")
        print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
        print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
        print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
        print("Particles > 2.5um / 0.1L air:", aqdata["particles 25um"])
        print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
        print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
        print("---------------------------------------")

    GREEN_PM_10 = 5
    YELLOW_PM_10 = 15

    # URLs to fetch from
    TEXT_URL = "http://wifitest.adafruit.com/testwifi/index.html"
    JSON_QUOTES_URL = "https://www.adafruit.com/api/quotes.php"
    JSON_STARS_URL = "https://api.github.com/repos/adafruit/circuitpython"

    def setFans(self, percent):
        for fan in self.fanspwm:
            self.setFanPWM(percent, fan)

    def setRGBbyPM(self, aqdata):
        pm10val = aqdata["pm100 standard"]
        if pm10val < self.GREEN_PM_10:
            self.setRGB(Color.GREEN)
            self.setFans(0.1)
        elif pm10val < self.YELLOW_PM_10:
            self.setRGB(Color.YELLOW)
            self.setFans(0.6)
        else:
            self.setRGB(Color.RED)
            self.setFans(1)
            
    def ctof(self, degrees):
        return (degrees * (9.0 / 5.0)) + 32

    def printData(self):
        scd_temp_meta = {
            'sensor': 'scd41',
            'units': 'fahrenheit',
        }
        scd_humidity_meta = {
            'sensor': 'scd41',
            'units': 'rh',
        }
        scd_co2_meta = {
            'sensor': 'scd41',
            'units': 'ppm',
        }
        if self.scd4x.data_ready:
            scd4xtemp = self.ctof(self.scd4x.temperature)
            print("CO2: %d ppm" % self.scd4x.CO2)
            print("Temperature: %0.1f *F" % scd4xtemp)
            print("Humidity: %0.1f %%" % self.scd4x.relative_humidity)
            print()
            self.aio.send_data(self.temperature_feed_scd["key"], scd4xtemp, scd_temp_meta)
            self.aio.send_data(self.humidity_feed["key"], self.scd4x.relative_humidity, scd_humidity_meta)
            self.aio.send_data(self.co2_feed["key"], self.scd4x.CO2, scd_co2_meta)
            
        else:
            print("SCD4X Data not ready!")

        bmp_temp_meta = {
            'sensor': 'bmp180',
            'units': 'fahrenheit',
        }
        bmp_pressure_meta = {
            'sensor': 'bmp180',
            'units': 'hPa',
        }
        bmptemp = self.ctof(self.bmp180.temperature)
        print("\nTemperature: %0.1f F" % bmptemp)
        print("Pressure: %0.1f hPa" % self.bmp180.pressure)
        print("Altitude = %0.2f meters" % self.bmp180.altitude)
        self.aio.send_data(self.temperature_feed_bmp["key"], bmptemp, bmp_temp_meta)
        self.aio.send_data(self.pressure_feed["key"], self.bmp180.pressure, bmp_pressure_meta)
        
        try:
            aqdata = self.pm25.read()
            self.printaqdata(aqdata)
            self.setRGBbyPM(aqdata)
            pm_meta = {
                'sensor': 'PMS7003',
                'units': 'ppm',
            }
            self.aio.send_data(self.pm25_feed["key"], aqdata["pm25 standard"], pm_meta)
            self.aio.send_data(self.pm10_feed["key"], aqdata["pm100 standard"], pm_meta)
        except RuntimeError as ex:
            print(ex)
            print("Unable to read from sensor, retrying...")

    def run(self):
        lastTime = time.monotonic() - 10000
        while True:
            if time.monotonic() - lastTime >= 30:
                lastTime = time.monotonic()
                self.printData()


biz = Business()
biz.run()
