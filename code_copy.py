""" Basic script for project code

This Python script demonstrates the recommended template for a project code.
It includes essential details, header, imports, parameters and variables,
functions and main code.

Please note the following quote by Guido van Rossum, the creator of Python

    “Code is more often read than written.”
    — Guido van Rossum

Author: John Doe
Created: 12-September-2024

"""

# Import needed modules, either with their current name or with a given nickname
import gc
import time
import wumons as wum
import rainbowio

from wumons.dfr_urm09 import DFRobot_URM09
from wumons.dfr_0991 import DFRobot_RGB_Button
from wumons.dfr_0646 import DFRobot_0646

from adafruit_mcp230xx.mcp23017 import MCP23017

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Other interesting modules can be identified on
#   https://docs.circuitpython.org/en/latest/shared-bindings/index.html
# for examples, math or random

# Define parameters
# ============================
#   these are values which will not change when the script is run,
#   but which you want to be able to change easily from one run to the next
TEST_SET = {"I2C", "MUSIC", "DISPLAY_DISTANCE"}
# {"HELP", "I2C", "MCP", "ADS", "DISPLAY", "NEOPIXEL", "MUSIC",
#    "RGB_BUTTON", "BUTTON", "MOVE", "DISTANCE", "DISPLAY_DISTANCE", }
SMALL_PAUSE = 0.05
BRIGHTNESS = 0.2  # Values in (0.0, 1.0), where 0.0 is off and 1.0 is maximum
RED = 0xFF0000  # Could be also written as (0xFF, 0, 0)
GREEN = 0x00FF00
BLUE = 0x0000FF
BLACK = 0x000000
WHITE = 0xFFFFFF

# Connect physical components
# ============================

#   Connect the two buttons of the device
button_a = wum.DigitalButton(wum.board.GP18)
button_b = wum.DigitalButton(wum.board.GP19)
#   Connect the two leds of the device
pixels = wum.NeoPixel(wum.board.GP22, 2, brightness=BRIGHTNESS, auto_write=False)
#   Connect the four possible motors
motor1 = wum.DCMotor(wum.board.GP20, wum.board.GP21)
motor2 = wum.DCMotor(wum.board.GP10, wum.board.GP11)
motor3 = wum.DCMotor(wum.board.GP14, wum.board.GP15)
motor4 = wum.DCMotor(wum.board.GP12, wum.board.GP13)
#   Connect the servos
servo0 = wum.Servo(wum.board.GP0)
servo1 = wum.Servo(wum.board.GP1)
servo2 = wum.Servo(wum.board.GP2)
servo3 = wum.Servo(wum.board.GP3)
# servo4 = wum.Servo(wum.board.GP4) # en conflit avec DCMotor1
# servo5 = wum.Servo(wum.board.GP5) # en conflit avec DCMotor1
servo6 = wum.Servo(wum.board.GP6)
servo7 = wum.Servo(wum.board.GP7)
# servo8 = wum.Servo(wum.board.GP8) # en conflit avec Music
# servo26 = wum.Servo(wum.board.GP26) # en conflit avec DCMotor2 ou Sensor0
# servo27 = wum.Servo(wum.board.GP27) # en conflit avec DCMotor2 ou Sensor1
# servo28 = wum.Servo(wum.board.GP28) # en conflit avec DCMotor4 ou Sensor2
#   Connect the analog sensors
sensor0 = wum.Sensor(wum.board.GP26)
sensor1 = wum.Sensor(wum.board.GP27)
sensor2 = wum.Sensor(wum.board.GP28)
#   Connect the sound player
music_player = wum.Music(wum.board.GP9)

# Connect and initialize the URM09 device
distance_sensor = DFRobot_URM09(wum.i2c)
# Connect the display device
display = DFRobot_0646(wum.i2c)  # DFR0645 afficheur LED 4 digit
# Connect the RGB button
rgb_button = DFRobot_RGB_Button(wum.i2c, 0x2A)


# Define functions
# ============================
def rainbow_cycle(leds, wait):
    """Colour animation on the LEDs

    Args:
        leds: the neopixel component
        wait: the cycle pause in seconds

    Returns:
        none
    """
    for color in range(255):
        for pixel in range(len(leds)):
            pixel_index = (pixel * 256 // len(leds)) + color * 5
            leds[pixel] = rainbowio.colorwheel(pixel_index & 255)
        leds.show()
        time.sleep(wait)

def rainbow_off(leds):
    """Switch the LEDs off

    Args:
        leds: the neopixel component
        wait: the cycle pause

    Returns:
        none
    """
    for pixel in range(len(leds)):
        leds[pixel] = BLACK
    leds.show()


def robot_forward(left_motor, right_motor, speedM):
    left_motor.set_speed(speedM)
    right_motor.set_speed(speedM)


def robot_backward(left_motor, right_motor, speedM):
    left_motor.set_speed(-speedM)
    right_motor.set_speed(-speedM)


def robot_turn_left(left_motor, right_motor, speedM):
    left_motor.set_speed(-speedM)
    right_motor.set_speed(speedM)


def robot_turn_right(left_motor, right_motor, speedM):
    left_motor.set_speed(speedM)
    right_motor.set_speed(-speedM)


def robot_stop(left_motor, right_motor):
    left_motor.set_speed(0)
    right_motor.set_speed(0)


# Write main code
# ============================
def main():
    startFlag = False
    
    motorL = motor2
    motorR = motor3
    my_start_speed = 0.6
    my_normal_speed = 0.5
    my_single_speed = 0.4

    time_step = 0.25

    distance_threshold = 3
    
    display.print("Ready !")
    rgb_button.set_RGB_color(0x0000FF)
    music_player.play(music_player.POWER_UP)
    while True:
        if (rgb_button.is_pressed()):
            startFlag = True
            rgb_button.set_RGB_color(0x00FF00)
            display.print("RUN")
            music_player.play(music_player.BA_DING)
            time.sleep(time_step)
        while startFlag:
            current_distance = distance_sensor.get_distance()
            display.int(current_distance)
            #
            if (current_distance<distance_threshold):
                rgb_button.set_RGB_color(0xFF8000)
                robot_backward(motorL, motorR, my_normal_speed)
                time.sleep(time_step)
                robot_turn_right(motorL, motorR, my_normal_speed)
                time.sleep(time_step)
            else:
                rgb_button.set_RGB_color(0x00FF00)
                robot_forward(motorL, motorR, my_normal_speed)
                time.sleep(time_step)
            #    
            if (rgb_button.is_pressed()):
                startFlag = False
                rgb_button.set_RGB_color(0xFF0000)
                display.print("STOP")
                robot_stop(motorL, motorR)
                music_player.play(music_player.WAWAWAWAA)
    time.sleep(5)
    
def oldmain():
    print("Hello World!")
    print(TEST_SET)

    if "HELP" in TEST_SET:
        print("Help test")
        wum.help()
        wum.motor.help()
    if "I2C" in TEST_SET:
        # I2C bus
        print("I2C test")
        while not wum.i2c.try_lock():
            pass
        try:
            print(
                "I2C addresses found:",
                [hex(device_address) for device_address in wum.i2c.scan()],
            )
        finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
            wum.i2c.unlock()
    if "ADS" in TEST_SET:
        # Analog-Digital converter
        print("ADS test")
        ads = ADS.ADS1115(wum.i2c, address=0x48)
        # Create single-ended input on channel 0
        chan = AnalogIn(ads, ADS.P0)
        # Create differential input between channel 0 and 1
        # chan = AnalogIn(ads, ADS.P0, ADS.P1)
        print("{:>5}\t{:>5}".format("raw", "v"))

        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            print("{:>5}\t{:>5.3f}".format(chan.value, chan.voltage))
            time.sleep(0.25)
    if "MCP" in TEST_SET:
        # 16 GPIO multiplexer
        print("MCP test")
        mcp = MCP23017(wum.i2c, address=0x27)

        pin = mcp.get_pin(7)
        pin.switch_to_input()
        mcp.get_pin(8).switch_to_input()

        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            value = 1 * pin.value
            print((value, mcp.gpio, mcp.gpioa))
            time.sleep(0.25)
    if "RGB_BUTTON" in TEST_SET:
        # RGB Button
        rgb_button = wum.DFRobot_RGB_Button(wum.i2c, 0x2A)
        rgb_button.set_RGB_color(0xFF0000)
        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            print(rgb_button.is_pressed())
            if rgb_button.is_pressed():
                rgb_button.set_RGB_color(0x00FF00)
            else:
                rgb_button.set_RGB_color(0xFF0000)
            time.sleep(0.5)
    if "BUTTON" in TEST_SET:
        # Button on card
        print("Button Test")
        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            print(button_b.is_pressed())
            time.sleep(0.5)
    if "DISPLAY" in TEST_SET:
        # 8-led display
        print("DISPLAY Test")
        display.int(4289)
        time.sleep(5)
        display.float(-3.14159)
        time.sleep(5)
        display.print("CircuitPython is great!")
        time.sleep(2)
        display.clear()
        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            for i in range(0, 6):
                for j in range(0, 8):
                    display.__set_raw_value(j, 2 ** ((i + j) % 6))
                display.__send_buf()
                time.sleep(0.5)
        display.off()
    if "DISTANCE" in TEST_SET:
        # Distance sensor
        print("Distance Test")
        # If needed, config the device
        # distance_sensor.set_mode_range(
        #    DFRobot_URM09.MEASURE_MODE_AUTOMATIC, DFRobot_URM09.MEASURE_RANG_500
        # )
        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            current_distance = distance_sensor.get_distance()
            print((current_distance,))
            time.sleep(0.5)
    if "DISPLAY_DISTANCE" in TEST_SET:
        print("DisplayDistance Test")
        display.print("I start")
        time.sleep(3)

        # If needed, config the device
        # distance_sensor.set_mode_range(
        #    DFRobot_URM09.MEASURE_MODE_AUTOMATIC, DFRobot_URM09.MEASURE_RANG_500
        # )

        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            current_distance = distance_sensor.get_distance()
            print((current_distance,))
            display.int(current_distance)
            time.sleep(0.25)
        display.print("I stop")
        time.sleep(3)
        display.off()
        
    if "NEOPIXEL" in TEST_SET:
        print("NeoPixel test")
        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            rainbow_cycle(pixels, SMALL_PAUSE)
        rainbow_off(pixels)
    if "MUSIC" in TEST_SET:
        print("Music test")
        music_player.play(music_player.CHASE)
    if "SENSOR" in TEST_SET:
        print("Sensor Test")
        start_time = time.monotonic()
        loop_duration = 10
        while time.monotonic() - start_time < loop_duration:
            print((sensor0.get_value(),))
            time.sleep(0.5)
    if "SERVO" in TEST_SET:
        print("Servo Test")
        start_time = time.monotonic()
        loop_duration = 10
        servoX = servo1
        while time.monotonic() - start_time < loop_duration:
            if button_a.is_pressed():
                robot_forward(0.8)
                servoX.set_angle(180)
                time.sleep(2)
                robot_stop()
                time.sleep(1)
                robot_backward(0.8)
                servoX.set_angle(0)
                time.sleep(2)
                robot_stop()
                time.sleep(1)

    if "MOVE" in TEST_SET:
        print("Move Test")
        start_time = time.monotonic()
        loop_duration = 30
        motorL = motor2
        motorR = motor3
        my_start_speed = 0.6
        my_normal_speed = 0.3
        my_single_speed = 0.4
        threshold_forward = 5
        while time.monotonic() - start_time < loop_duration:
            current_distance = distance_sensor.get_distance()
            print(
                (
                    current_distance,
                    threshold_forward,
                )
            )
            if current_distance > threshold_forward:
                robot_forward(motorL, motorR, my_start_speed)
                time.sleep(0.05)
                robot_forward(motorL, motorR, my_normal_speed)
            else:
                robot_stop(motorL, motorR)
                robot_backward(motorL, motorR, my_single_speed)
                time.sleep(0.5)
                robot_turn_right(motorL, motorR, my_single_speed)
                time.sleep(0.25)
                robot_stop(motorL, motorR)
            time.sleep(1)


if __name__ == "__main__":
    main()
    print(gc.mem_free())
