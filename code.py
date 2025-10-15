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
#   Connect the sound player
music_player = wum.Music(wum.board.GP9)

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

    
def main():
    print("Hello World!")
    start_time = time.monotonic()
    loop_duration = 10
    while time.monotonic() - start_time < loop_duration:
        rainbow_cycle(pixels, SMALL_PAUSE)
    rainbow_off(pixels)
    music_player.play(music_player.CHASE)
    

if __name__ == "__main__":
    main()
    print(gc.mem_free())
