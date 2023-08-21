"""
main.py - Main program for the project.

This module contains the main program logic for the project. It initializes the hardware,
handles recording, and processes sensor data to detect moves.

Imports:
    sys: Provides functions for interacting with the console.
    time: Provides time-related functions.
    spike_detection: Module for spike detection and move validation.
    drone_motion_tools: Module for drone motion-related functions.
    init_hardware: Module for initializing hardware components.

Global Variables:
    sequence (dict): Dictionary to store sensor data sequences.
    final_sequence (list): List to store the final sequence of detected moves.
    init (bool): Flag indicating whether hardware initialization is complete.

Main Loop:
    The main loop continuously handles button inputs, recording, and move validation.

"""

import time
from spike_detect import check_sequence
from drone_motion_tools import *
from init_hardware import (
    init_hardware,
    sensor,
    ready_led,
    recording_led,
    correct_led,
    start_btn,
    stop_btn
)

# Dictionary to store sensor data sequences
sequence = {
    "AX": [], "AY": [], "AZ": [],
    "GX": [], "GY": [], "GZ": [],
}

# List to store the final sequence of detected moves
final_sequence = []

# Flag indicating whether hardware initialization is complete
init = False

while True:
    if not init:
        init_hardware()
        print("Started Program")

        # Initial state
        ready_led.value = True
        recording_led.value = False
        is_recording = False

    # Update states for stop/start buttons
    if start_btn.value and not is_recording:
        is_recording = True
        ready_led.value = False
        recording_led.value = True

    elif stop_btn.value and is_recording:
        is_recording = False
        ready_led.value = True
        recording_led.value = False

        # Validate move
        valid_moves = check_sequence(sequence)
        add_moves_to_sequence(valid_moves)

        # Reset the sequence for the next recording
        sequence = {
            "AX": [], "AY": [], "AZ": [],
            "GX": [], "GY": [], "GZ": [],
        }
        final_sequence = []
        pico_id = None
        print("\nWaiting for pico_id from client")

    # Recording
    if is_recording:

        # Prevent overflow (sequence terminates if trying to record for more than 10 seconds)
        if len(sequence["AX"]) > 1000:
            print("\n\n\n\n\n\n\n\nRestarting, overflowed 10s\n\n")
            sequence = {
                "AX": [], "AY": [], "AZ": [],
                "GX": [], "GY": [], "GZ": [],
            }

        add_all_sensor_data(sequence)

        print((round(sensor.acceleration[0], 1), round(sensor.acceleration[1], 1), round(sensor.acceleration[2] - z_offset, 1), sensitivity, -1 * sensitivity))

    time.sleep(0.1)
