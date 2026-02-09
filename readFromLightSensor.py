import smbus
import time
import webcolors

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

def getAndUpdateColour():
    # while True:
	# Read the data from the sensor
        # Insert code here
        data = bus.read_i2c_block_data(i2c_address, 0x09, 6)

        # Convert the data to green, red and blue int values
        # Insert code here
        red = data[3] << 8 | data[2]
        green = data[1] << 8 | data[0]
        blue = data[5] << 8 | data[4]

        red *= 2**(-8)
        green *= 2**(-8)
        blue *= 2**(-8)

        colors = [red, green, blue]
        time.sleep(1) 
        return colors

        # # Output data to the console RGB values
        # # Uncomment the line below when you have read the red, green and blue values
        # print("RGB(%d %d %d)" % (red, green, blue))
        

def closest_colour(requested_colour):
    distances = {}
    for name in webcolors.names():
        r_c, g_c, b_c = webcolors.name_to_rgb(name)
        rd = (r_c - requested_colour[0]) ** 2
        gd = (g_c - requested_colour[1]) ** 2
        bd = (b_c - requested_colour[2]) ** 2
        distances[name] = rd + gd + bd
    return min(distances, key=distances.get)

def get_colour_name(requested_colour):
    try:
        closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
    except ValueError:
        closest_name = closest_colour(requested_colour)
        actual_name = None
    return actual_name, closest_name

while True:
    colors = getAndUpdateColour()
    actual_name, closest_name = get_colour_name(colors)

    print("Actual colour name:", actual_name, ", closest colour name:", closest_name, " red:", colors[0], " green:", colors[1], " blue:", colors[2])