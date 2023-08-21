"""
init_hardware.py - Initialize hardware components for the project.

This module initializes push buttons, LEDs, and the IMU sensor for the project.
It provides the `init_hardware()` function to perform the necessary hardware setup.

Hardware Configuration:
                             USB
                         +----------+
                         |    +Y    |
                         |          |   Button: Start Recording
                         |          |
                         |          |   Button: Stop Recording
                         |          |
                         | -X    +X |
                         |          |
                         |          |
 LED: READY TO RECORD    |          |   LED: Sequence Status
                         |          |
 LED: RECORDING          |          |
                         |          |
                         |    -Y    |
                         +----------+

- if looking at pico from birds eye witht the usb pointing forward
- gyro x is pitch / front back
- gyro y is roll / side to side
- gyro z is yaw / clock counter clock

Functions:
    init_hardware(): Initialize hardware components.
"""

# Import necessary libraries
import time

import board
from digitalio import DigitalInOut, Direction, Pull
import busio
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33

# Global variable to track initialization status
init = False

def init_hardware():
    """
    Initialize hardware components.

    Initializes push buttons, LEDs, and the IMU sensor.
    """
    global init

    # Initialize push buttons
    start_btn = DigitalInOut(board.GP28)    # Upper button
    start_btn.direction = Direction.INPUT
    start_btn.pull = Pull.DOWN

    stop_btn = DigitalInOut(board.GP22)     # Lower button
    stop_btn.direction = Direction.INPUT
    stop_btn.pull = Pull.DOWN

    # Initialize status LEDs
    ready_led = DigitalInOut(board.GP12)       # Left upper LED
    ready_led.direction = Direction.OUTPUT

    recording_led = DigitalInOut(board.GP11)   # Left lower LED
    recording_led.direction = Direction.OUTPUT

    correct_led = DigitalInOut(board.GP16)    # Bottom right LED
    correct_led.direction = Direction.OUTPUT

    # Initialize onboard LED
    onboard_led = DigitalInOut(board.LED)
    onboard_led.direction = Direction.OUTPUT

    # Initialize I2C and LSM6DS33 sensor
    i2c = busio.I2C(scl=board.GP1, sda=board.GP0)
    sensor = LSM6DS33(i2c)

    # Blink LEDs to indicate initialization
    for _ in range(3):
        ready_led.value = recording_led.value = correct_led.value = onboard_led.value = True
        time.sleep(0.1)
        ready_led.value = recording_led.value = correct_led.value = onboard_led.value = False
        time.sleep(0.1)

    init = True
