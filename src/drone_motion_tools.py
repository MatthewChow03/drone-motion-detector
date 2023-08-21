"""
drone_motion_tools.py - Helper functions for the project.

This module contains utility functions for the project.

Functions:
    sign(num): Returns the sign of a number.
    sequence_correct_led(): Blinks the correct LED to indicate a valid sequence.
    add_all_sensor_data(sequence): Adds sensor data to a sequence dictionary.
    add_moves_to_sequence(valid_moves): Adds valid moves to the final sequence.
    print_all_imu(): Prints accelerometer and gyro data.
"""

import sys
import time
from init_hardware import correct_led


def sign(num):
    """
    Returns the sign of a number.

    Args:
        num (float): The input number.

    Returns:
        int: 1 if num is positive, -1 if num is negative, else 0.
    """
    if num == 0:
        return 1
    return num / abs(num)


def sequence_correct_led():
    """
    Blinks the correct LED to indicate a valid sequence.
    
    Modifies:
        correct_led: Blinks the correct LED.
    """
    for i in range(1):
        correct_led.value = True
        time.sleep(0.1)
        correct_led.value = False
        time.sleep(0.1)

    # Reset the correct LED after blinking
    correct_led.value = False

def add_all_sensor_data(sequence):
    """
    Adds sensor data to a sequence dictionary.

    Args:
        sequence (dict): The sequence dictionary.

    Modifies:
        sequence: Updates the AX, AY, AZ, GX, GY, and GZ lists with sensor data.
    """
    sequence["AX"].append(round(sensor.acceleration[0], 1))
    sequence["AY"].append(round(sensor.acceleration[1], 1))
    sequence["AZ"].append(round(sensor.acceleration[2], 1))
    sequence["GX"].append(round(sensor.gyro[0], 1))
    sequence["GY"].append(round(sensor.gyro[1], 1))
    sequence["GZ"].append(round(sensor.gyro[2], 1))

def add_moves_to_sequence(valid_moves):
    """
    Adds valid moves to the final sequence.

    Args:
        valid_moves (list): List of valid move strings.

    Modifies:
        final_sequence: Updates the final_sequence list with valid moves.
    """
    for move in valid_moves:
        if move == "FLIP":
            sequence_correct_led()
            print(move)
            final_sequence.append("FLIP")
        # ... other move cases ...

def print_all_imu():
    """
    Prints accelerometer and gyro data.

    Displays accelerometer and gyro data on the screen.

    Modifies:
        None
    """
    sys.stderr.write("\x1b[2J\x1b[0;0H")  # Clear the screen
    print("Acceleration (m/s^2)")
    print("------------------------\n")
    print("X:\t%6.1f\n" % (sensor.acceleration[0]))
    print("Y:\t%6.1f\n" % (sensor.acceleration[1]))
    print("Z:\t%6.1f\n" % (sensor.acceleration[2]))
    print()
    print("Gyro (rad/s)")
    print("------------------------\n")
    print("X:\t%6.1f\n" % (sensor.gyro[0]))
    print("Y:\t%6.1f\n" % (sensor.gyro[1]))
    print("Z:\t%6.1f\n" % (sensor.gyro[2]))

    time.sleep(0.05)
