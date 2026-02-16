import smbus
import time
import RPi.GPIO as GPIO
from gpiozero import LED

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 15

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

# Get I2C bus
bus = smbus.SMBus(1) # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
i2c_address = 0x44
bus.write_byte_data(i2c_address, 0x01, 0x05)

time.sleep(1)


print("Reading colour values and displaying them in a new window\n")

RED_H = 0x0C
RED_L = 0x0B

GREEN_H = 0x0A
GREEN_L = 0x09

BLUE_H = 0x0E
BLUE_L = 0x0D

def getDistance():
      # This function measures a distance
  # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO, False)
  #ensure start time is set in case of very quick return
    start = time.time()

  # set line to input to check for start of echo response
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO)==0:
        start = time.time()

  # Wait for end of echo response
    while GPIO.input(GPIO_TRIGECHO)==1:
        stop = time.time()
  
    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    # time.sleep(0.1)
    return distance

def getAndUpdateColour():
    # while True:
	# Read the data from the sensor
    data = bus.read_i2c_block_data(i2c_address, 0x09, 6)

    # upshift 
    red = data[3] << 8 | data[2]
    green = data[1] << 8 | data[0]
    blue = data[5] << 8 | data[4]

    red *= 2**(-8)
    green *= 2**(-8)
    blue *= 2**(-8)

    colors = [red, green, blue]
    return colors

#distance in cm
def updateLED(led:LED, distance):
    global LEDTime
    if distance > 140:
        swaptime = 3*(10**9)   # 3 seconds
    elif distance > 50:
        swaptime = 0.5*(10**9) # 0.5 second
    else:
        led.on()               # always on
        return

    if time.time_ns()-LEDTime > swaptime:
        LEDTime = time.time_ns()
        if led.value == 1:
            led.off()
        else:
            led.on()

try:
    led = LED(17)
    LEDTime = time.time_ns()
    while True:
        colors = getAndUpdateColour()

        print(f"red: {round(colors[0]):<5}", f"green: {round(colors[1]):< 5}", f"blue: {round(colors[2]):<5}\n")
        distance = getDistance()
        print("Distance : %.1f cm" % distance)
        updateLED(led, distance)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()