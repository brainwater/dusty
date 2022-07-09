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


RED = (1, 0, 0)
GREEN = (0, 1, 0)
BLUE = (0, 0, 1)
reset_pin = None


def setFanPWM(percent, pin):
    #rgbLed = tuple([pwmio.PWMOut(led) for led in [board.SCK, board.A3, board.A2]])
    #for led in rgbLed:
    #    led.duty_cycle = 2**12-1
    if percent > 1 or percent < 0:
        print("ERROR! percent must be between 0 and 1")
        return
    pin.duty_cycle = int(percent*(2**16-1))

def setRGB(rgbValue, percent=0.1):
    for led, val in zip(rgbLed, rgbValue):
        led.duty_cycle = int((2**16-1) * percent * val)

def printaqdata(aqdata):
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
GREEN = (0, 1, 0)
YELLOW = (1, 0.4, 0)
RED = (1, 0, 0)
BLUE = (0, 0, 1)

# URLs to fetch from
TEXT_URL = "http://wifitest.adafruit.com/testwifi/index.html"
JSON_QUOTES_URL = "https://www.adafruit.com/api/quotes.php"
JSON_STARS_URL = "https://api.github.com/repos/adafruit/circuitpython"

def setFans(percent):
    for fan in fanspwm:
        setFanPWM(percent, fan)

def setRGBbyPM(aqdata):
    pm10val = aqdata["pm100 standard"]
    if pm10val < GREEN_PM_10:
        setRGB(GREEN)
        setFans(0.1)
    elif pm10val < YELLOW_PM_10:
        setRGB(YELLOW)
        setFans(0.6)
    else:
        setRGB(RED)
        setFans(1)

def printData():
    if scd4x.data_ready:
        print("CO2: %d ppm" % scd4x.CO2)
        print("Temperature: %0.1f *C" % scd4x.temperature)
        print("Humidity: %0.1f %%" % scd4x.relative_humidity)
        print()
    else:
        print("SCD4X Data not ready!")

    print("\nTemperature: %0.1f C" % bmp180.temperature)
    print("Pressure: %0.1f hPa" % bmp180.pressure)
    print("Altitude = %0.2f meters" % bmp180.altitude)

    try:
        aqdata = pm25.read()
        printaqdata(aqdata)
        setRGBbyPM(aqdata)
    except RuntimeError:
        print("Unable to read from sensor, retrying...")

i2c = board.I2C()
scd4x = adafruit_scd4x.SCD4X(i2c)
print("SCD4X Serial number:", [hex(i) for i in scd4x.serial_number])
scd4x.start_periodic_measurement()

bmp180 = adafruit_bmp180.Adafruit_BMP180_I2C(i2c)
bmp180.sea_level_pressure = 1013.25 # TODO: Can I use this for updated local weather from the internet? or what?

rgbLed = tuple([pwmio.PWMOut(led) for led in [board.SCK, board.A3, board.A2]])
for led in rgbLed:
    led.duty_cycle = 2**12-1
time.sleep(0.4)
uart = busio.UART(board.TX, board.RX, baudrate=9600)
pm25 = PM25_UART(uart, reset_pin)
print("Found PM2.5 sensor, reading data...")

fanspwm = tuple([pwmio.PWMOut(pin, frequency=25*(10**3)) for pin in [board.MISO, board.A0]])

print("Available WiFi networks:")
for network in wifi.radio.start_scanning_networks():
    print("\t%s\t\tRSSI: %d\tChannel: %d" % (str(network.ssid, "utf-8"),
            network.rssi, network.channel))
wifi.radio.stop_scanning_networks()
print("Connecting to %s"%secrets["ssid"])
wifi.radio.connect(secrets["ssid"], secrets["password"])
print("Connected to %s!"%secrets["ssid"])
pool = socketpool.SocketPool(wifi.radio)
requests = adafruit_requests.Session(pool, ssl.create_default_context())

adafruitIO = IO_HTTP(secrets["ADAFRUIT_IO_USERNAME"], secrets["ADAFRUIT_IO_KEY"], requests)

#print("My IP address is", wifi.radio.ipv4_address)
#ipv4 = ipaddress.ip_address("8.8.4.4")
#print("Ping google.com: %f ms" % (wifi.radio.ping(ipv4)*1000))


#print("Fetching text from", TEXT_URL)
#response = requests.get(TEXT_URL)
#print("-" * 40)
#print(response.text)
#print("-" * 40)

location_id = 2552

print("Getting forecast from IO...")
# Fetch the specified record with current weather
# and all available forecast information.
forecast = adafruitIO.receive_weather(location_id)
# Get today's forecast
current_forecast = forecast["current"]
lastTime = time.monotonic()



while True:
    if time.monotonic() - lastTime >= 30:
        lastTime = time.monotonic()
        printData()
        print(
            "It is {0} and {1}*F.".format(
                current_forecast["summary"], current_forecast["temperature"]
            )
        )
        print("with a humidity of {0}%".format(current_forecast["humidity"] * 100))

        # Get tomorrow's forecast
        tom_forecast = forecast["forecast_days_1"]
        print(
            "\nTomorrow has a low of {0}*F and a high of {1}*F.".format(
                tom_forecast["temperatureLow"], tom_forecast["temperatureHigh"]
            )
        )
        print("with a humidity of {0}%".format(tom_forecast["humidity"] * 100))

