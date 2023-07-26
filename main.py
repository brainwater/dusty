import board
import pwmio
import time
import busio
import adafruit_scd4x
import adafruit_bmp180
import ssl
import wifi
import socketpool
import adafruit_requests
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import json
#from rainbowio import colorwheel
import neopixel
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
    RED = (255, 0, 0)
    LIGHT_RED = (100, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    OFF = (0, 0, 0)

    YELLOW = (255, 102, 0)

class Business:
    reset_pin = None
    GREEN_PM_10 = 5
    YELLOW_PM_10 = 15
    GREEN_CO2 = 600
    YELLOW_CO2 = 1000
    RED_CO2 = 1400

    def __init__(self):
        self.fanspwm = tuple([pwmio.PWMOut(pin, frequency=25*(10**3)) for pin in [board.MISO, board.A0]])
        self.rgbLed = tuple([pwmio.PWMOut(led) for led in [board.SCK, board.A3, board.A2]])
        self.pixels = neopixel.NeoPixel(board.SDA, 7, brightness=0.05)
        self.pixels.fill(Color.BLUE)
        for led in self.rgbLed:
            led.duty_cycle = 2**12-1
        self._initialize()

    # Used to prevent intermittent voltage drop on startup that resets board
    def _spinFansUp(self):
        for i in range(90):
            time.sleep(0.05)
            self.setFans(i / 100.0)

    def _initBMP(self):
        i2c = board.STEMMA_I2C()
        self.bmp180 = adafruit_bmp180.Adafruit_BMP180_I2C(i2c)
        self.bmp180.sea_level_pressure = 1013.25 # TODO: Can I use this for updated local weather from the internet? or what?
        print("Initialized BMP180 pressure and temperature sensor")

    def _initSCD4X(self):
        i2c = board.STEMMA_I2C()
        self.scd4x = adafruit_scd4x.SCD4X(i2c)
        print("SCD4X Serial number:", [hex(i) for i in self.scd4x.serial_number])
        self.scd4x.start_periodic_measurement()
        print("Initalized SCD4X CO2 Sensor")

    def _initPM25(self):
        uart = busio.UART(board.TX, board.RX, baudrate=9600)
        self.pm25 = PM25_UART(uart, self.reset_pin)
        print("Found PM2.5 sensor, reading data...")

    def _initNetwork(self):
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
        self.mqtt_client = MQTT.MQTT(
            broker=secrets["mqtt_broker"],
            port=secrets["mqtt_port"],
            username=secrets["mqtt_username"],
            password=secrets["mqtt_password"],
            socket_pool=pool,
            ssl_context=ssl.create_default_context(),
        )
        print("Attempting to connect to %s" % self.mqtt_client.broker)
        self.mqtt_client.connect()
        self.aio = IO_HTTP(secrets["ADAFRUIT_IO_USERNAME"], secrets["ADAFRUIT_IO_KEY"], requests)

    def _initSensor(self, device_class: str, prefix: str, unit_of_measurement: str):
        """prefix = prefix.lower().strip()
        prefix_up = prefix[:1].upper() + prefix[1:] # str.capitalize() doesn't exist here for some reason
        device_class = device_class.lower().strip()
        device_class_up = device_class[:1].upper() + device_class[1:]
        payload = {
            "name": prefix_up + " " + device_class_up,
            "device_class": device_class,
            "state_topic": "homeassistant/sensor/" + prefix + device_class + "/state",
            "unit_of_measurement": unit_of_measurement,
            "payload_available": "online",
            "payload_not_available": "offline",
            "unique_id": prefix + device_class + "gauge",
            "suggested_display_precision": "1",
            "value_template": "{{ value_json." + device_class + " | round(1) }}"
        }
        topic = "homeassistant/sensor/" + prefix + device_class + "/config"
        self.mqtt_client.reconnect()
        self.mqtt_client.publish(topic, json.dumps(payload))"""
        pass

    def _advertiseSensor(self, device_class: str, prefix: str, unit_of_measurement: str):
        prefix = prefix.lower().strip()
        prefix_up = prefix[:1].upper() + prefix[1:] # str.capitalize() doesn't exist here for some reason
        device_class = device_class.lower().strip()
        device_class_up = device_class[:1].upper() + device_class[1:]
        payload = {
            "name": prefix_up + " " + device_class_up,
            "device_class": device_class,
            "state_topic": "homeassistant/sensor/" + prefix + device_class + "/state",
            "unit_of_measurement": unit_of_measurement,
            "payload_available": "online",
            "payload_not_available": "offline",
            "unique_id": prefix + device_class + "gauge",
            "suggested_display_precision": "1",
            "value_template": "{{ value_json." + device_class + " | round(1) }}"
        }
        topic = "homeassistant/sensor/" + prefix + device_class + "/config"
        self.mqtt_client.reconnect()
        self.mqtt_client.publish(topic, json.dumps(payload))

    def _publishSensor(self, device_class: str, prefix: str, value):
        prefix = prefix.lower().strip()
        device_class = device_class.lower().strip()
        topic = "homeassistant/sensor/" + prefix + device_class + "/state"
        output = {
            device_class: value
        }
        self.mqtt_client.reconnect()
        self.mqtt_client.publish(topic, json.dumps(output))

    def _initFeeds(self):
        self._initSensor("temperature", "dustybmp180", "°F")
        self._initSensor("temperature", "dustyscd41", "°F")
        self._initSensor("humidity", "dusty", "%rH")
        self._initSensor("pressure", "dusty", "hPa")
        self._initSensor("carbon_dioxide", "dusty", "ppm")
        self._initSensor("pm25", "dusty", "ppm")
        self._initSensor("pm10", "dusty", "ppm")
        self.temperature_feed_bmp = self.aio.get_feed('dusty.temperature-bmp180')
        self.temperature_feed_scd = self.aio.get_feed('dusty.temperature-scd41')
        self.pressure_feed = self.aio.get_feed('dusty.pressure')
        self.co2_feed = self.aio.get_feed('dusty.co2')
        self.humidity_feed = self.aio.get_feed('dusty.humidity')
        self.pm25_feed = self.aio.get_feed('dusty.pm25')
        self.pm10_feed = self.aio.get_feed('dusty.pm10')

    def _criticalFun(self, fun, error="ERROR In Critical Function"):
        try:
            fun()
        except Exception as ex:
            print(error)
            print(type(ex))
            print(ex)
            print(dir(ex))
            self.errorState()

    def _initialize(self):
        self._spinFansUp()
        self._criticalFun(self._initSCD4X)
        self._criticalFun(self._initBMP)
        time.sleep(0.4)
        self._criticalFun(self._initPM25)
        self._criticalFun(self._initNetwork)
        self._criticalFun(self._initFeeds)

    def errorState(self):
        while True:
            self.setRGBParticulates(Color.YELLOW, 1.0)
            time.sleep(0.5)
            self.setRGBParticulates(Color.RED, 1.0)
            time.sleep(0.5)
            self.setRGBParticulates(Color.OFF, 0.0)
            time.sleep(0.5)

    def setFanPWM(self, percent, pin):
        if percent > 1 or percent < 0:
            print("ERROR! percent must be between 0 and 1")
            return
        pin.duty_cycle = int(percent*(2**16-1))

    def setRGBParticulates(self, rgbValue, percent=0.1):
        for led, val in zip(self.rgbLed, rgbValue):
            led.duty_cycle = int((2**16-1) * percent * val / 256.0)
        self.pixels[0] = rgbValue

    def setRGBCO2(self, rgbValue):
        self.pixels[1] = rgbValue

    def printaqdata(self, aqdata):
        print()
        print("Concentration Units (standard)")
        print(
            "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
            % (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"])
        )
        print("---------------------------------------")
        print("Concentration Units (environmental)")
        print(
            "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
                % (aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"])
        )
        print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
        print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
        print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
        print("Particles > 2.5um / 0.1L air:", aqdata["particles 25um"])
        print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
        print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
        print()

    def setFans(self, percent):
        for fan in self.fanspwm:
            self.setFanPWM(percent, fan)

    def setRGBbyPM(self, aqdata):
        pm10val = aqdata["pm100 standard"]
        if pm10val < self.GREEN_PM_10:
            self.setRGBParticulates(Color.GREEN)
            self.setFans(0.1)
        elif pm10val < self.YELLOW_PM_10:
            self.setRGBParticulates(Color.YELLOW)
            self.setFans(0.6)
        else:
            self.setRGBParticulates(Color.RED)
            self.setFans(1)

    def setRGBbyCO2(self, co2ppm):
        if co2ppm < self.GREEN_CO2:
            self.setRGBCO2(Color.GREEN)
        elif co2ppm < self.YELLOW_CO2:
            self.setRGBCO2(Color.YELLOW)
        elif co2ppm < self.RED_CO2:
            self.setRGBCO2(Color.LIGHT_RED)
        else:
            self.setRGBCO2(Color.RED)

    def ctof(self, degrees):
        return (degrees * (9.0 / 5.0)) + 32

    def _printSCD(self):
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
            self.setRGBbyCO2(self.scd4x.CO2)
            print("CO2: %d ppm" % self.scd4x.CO2)
            print("Temperature: %0.1f *F" % scd4xtemp)
            print("Humidity: %0.1f %%" % self.scd4x.relative_humidity)
            print()
            self._advertiseSensor("temperature", "dustyscd41", "°F")
            self._advertiseSensor("humidity", "dusty", "%rH")
            self._advertiseSensor("carbon_dioxide", "dusty", "ppm")
            self._publishSensor("temperature", "dustyscd41", scd4xtemp)
            self._publishSensor("humidity", "dusty", self.scd4x.relative_humidity)
            self._publishSensor("carbon_dioxide", "dusty", self.scd4x.CO2)
            self.aio.send_data(self.temperature_feed_scd["key"], scd4xtemp, scd_temp_meta)
            self.aio.send_data(self.humidity_feed["key"], self.scd4x.relative_humidity, scd_humidity_meta)
            self.aio.send_data(self.co2_feed["key"], self.scd4x.CO2, scd_co2_meta)
        else:
            print("SCD4X Data not ready!")
            self.scd4x.start_periodic_measurement()

    def _printBMP(self):
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
        self._advertiseSensor("pressure", "dusty", "hPa")
        self._advertiseSensor("temperature", "dustybmp180", "°F")
        self._publishSensor("temperature", "dustybmp180", bmptemp)
        self._publishSensor("pressure", "dusty", self.bmp180.pressure)
        self.aio.send_data(self.temperature_feed_bmp["key"], bmptemp, bmp_temp_meta)
        self.aio.send_data(self.pressure_feed["key"], self.bmp180.pressure, bmp_pressure_meta)

    def _printPM25(self):
        aqdata = self.pm25.read()
        self.printaqdata(aqdata)
        self.setRGBbyPM(aqdata)
        pm_meta = {
            'sensor': 'PMS7003',
            'units': 'ppm',
        }
        self._advertiseSensor("pm25", "dusty", "ppm")
        self._advertiseSensor("pm10", "dusty", "ppm")
        self._publishSensor("pm25", "dusty", aqdata["pm25 standard"])
        self._publishSensor("pm10", "dusty", aqdata["pm100 standard"])
        self.aio.send_data(self.pm25_feed["key"], aqdata["pm25 standard"], pm_meta)
        self.aio.send_data(self.pm10_feed["key"], aqdata["pm100 standard"], pm_meta)

    def _noncriticalFun(self, fun):
        try:
            fun()
        except Exception as ex:
            print("Noncritical exception, either a transient fault or a disconnected sensor.")
            print(ex)

    def printData(self):
        self._noncriticalFun(self._printSCD)
        self._noncriticalFun(self._printBMP)
        self._noncriticalFun(self._printPM25)

    def run(self):
        lastTime = time.monotonic() - 10000
        while True:
            if time.monotonic() - lastTime >= 30:
                lastTime = time.monotonic()
                self._criticalFun(self.printData)
            time.sleep(1.0)


biz = Business()
biz.run()
