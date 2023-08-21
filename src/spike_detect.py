"""
spike_detect.py - Module for spike detection and move validation.

This module contains functions to detect valid moves in the sensor data sequence,
perform spike detection, and process the sequence to generate the final sequence
of detected moves.

Global Variables:
    sensitivity (int): Sensitivity threshold for motion detection.
    buffer_offset (int): Offset value for buffer adjustment.
    z_offset (int): Offset value for Z-axis sensor data.

Functions:
    apply_buffer(buffer, offset): Apply buffer adjustment to a value.
    apply_offset(val, offset): Apply offset to a value.
    is_positive_motion(val, threshold): Check if a value indicates positive motion.
    is_negative_motion(val, threshold): Check if a value indicates negative motion.
    is_move_within_buffer(buffer, threshold): Check if a move is within the buffer zone.
    get_valid_moves(sequence, sensor_values, axis_name, move_type, threshold, offset=0): Get valid moves based on sensor values.
    check_sequence(sequence): Detect valid moves in the sensor data sequence.
    process_valid_moves(valid_moves_indexed): Process and filter valid moves to generate the final sequence.
    find_max_moves(valid_moves_indexed): Find and filter the maximum moves within the valid moves.
"""

# Global parameters
sensitivity = 4
buffer_offset = 4
z_offset = 10

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------

def apply_buffer(buffer, offset):
    """
    Apply buffer adjustment to a value.

    Args:
        buffer (int): The buffer value.
        offset (int): The offset value.

    Returns:
        int: Updated buffer value.
    """
    if buffer < 0:
        buffer = 0
    elif buffer > 0:
        buffer = buffer - 1
    return buffer

def apply_offset(val, offset):
    """
    Apply offset to a value.

    Args:
        val (float): The value to apply offset to.
        offset (float): The offset value.

    Returns:
        float: Updated value with offset.
    """
    return val - offset

def is_positive_motion(val, threshold):
    """
    Check if a value represents positive motion.

    Args:
        val (float): The value to check.
        threshold (float): The threshold value for positive motion.

    Returns:
        bool: True if the value indicates positive motion, False otherwise.
    """
    return val > threshold

def is_negative_motion(val, threshold):
    """
    Check if a value represents negative motion.

    Args:
        val (float): The value to check.
        threshold (float): The threshold value for negative motion.

    Returns:
        bool: True if the value indicates negative motion, False otherwise.
    """
    return val < -threshold

def is_move_within_buffer(buffer, threshold):
    """
    Check if a buffer value indicates that a move is within the buffer zone.

    Args:
        buffer (int): The buffer value.
        threshold (int): The threshold value.

    Returns:
        bool: True if the move is within the buffer, False otherwise.
    """
    return buffer == 0

def get_valid_moves(sequence, sensor_values, axis_name, move_type, threshold, offset=0):
    """
    Get valid moves based on sensor values.

    Args:
        sequence (dict): The sensor data sequence.
        sensor_values (list): The sensor values for a specific axis.
        axis_name (str): The name of the axis.
        move_type (str): The type of move being checked.
        threshold (float): The threshold value for valid motion.
        offset (float, optional): Offset to apply to sensor values. Defaults to 0.

    Returns:
        list: List of valid moves.
    """
    valid_moves = []
    buffer = 0

    for i, val in enumerate(sensor_values):
        buffer = apply_buffer(buffer, buffer_offset)

        if is_move_within_buffer(buffer, threshold):
            if move_type == "UP" or move_type == "DOWN":
                val = apply_offset(val, offset)
            if is_positive_motion(val, threshold):
                valid_moves.append((move_type, i, val))
                buffer += buffer_offset

    return valid_moves

# -----------------------------------------------------------------------------
# IMU DATA PARSING -> SEQUENCE
# -----------------------------------------------------------------------------

def check_sequence(sequence):
    """
    Detect valid moves in the sensor data sequence.

    Args:
        sequence (dict): The sensor data sequence.

    Returns:
        list: List of detected valid moves.
    """
    valid_moves_indexed = []

    # Check for flip motion
    started_up = False
    for i, z in enumerate(sequence["AZ"]):
        buffer = apply_buffer(0, 0)  # Reset buffer for each iteration

        if is_move_within_buffer(buffer, 0):
            if z >= 0:
                started_up = True
            elif z < 0 and started_up:
                if (i + 1) < (len(sequence["AZ"]) - 1) and (i - 1) > 0:
                    flip = True
                    for j in range(i - 1, i + 1 + 1):
                        if sequence["AZ"][j] > 0:
                            flip = False
                    if flip:
                        valid_moves_indexed.append(("FLIP", i, 0))
                        buffer = buffer + buffer_offset
                        started_up = False

    valid_moves_indexed.extend(get_valid_moves(sequence, sequence["AX"], "AX", "RIGHT", sensitivity))
    valid_moves_indexed.extend(get_valid_moves(sequence, sequence["AY"], "AY", "FORWARD", sensitivity))
    valid_moves_indexed.extend(get_valid_moves(sequence, sequence["AZ"], "AZ", "UP", sensitivity, z_offset))
    valid_moves_indexed.extend(get_valid_moves(sequence, sequence["AX"], "AX", "LEFT", sensitivity))
    valid_moves_indexed.extend(get_valid_moves(sequence, sequence["AY"], "AY", "BACKWARD", sensitivity))
    valid_moves_indexed.extend(get_valid_moves(sequence, sequence["AZ"], "AZ", "DOWN", sensitivity, z_offset))

    return process_valid_moves(valid_moves_indexed)

# -----------------------------------------------------------------------------
# SEQUENCE PROCESSING
# -----------------------------------------------------------------------------

def process_valid_moves(valid_moves_indexed):
    """
    Process and filter valid moves to generate the final sequence of detected moves.

    Args:
        valid_moves_indexed (list): List of indexed valid moves.

    Returns:
        list: Final sequence of detected moves.
    """
    sorted_moves = []
    tolerance = 4

    valid_moves_indexed.sort(key=lambda x: x[1])

    flip_occurrence_indices = [pair[1] for pair in valid_moves_indexed if pair[0] == "FLIP"]

    moves_to_remove = []
    for index in flip_occurrence_indices:
        for move_index, pair in enumerate(valid_moves_indexed):
            if abs(pair[1] - index) < tolerance and pair[0] != "FLIP":
                moves_to_remove.append(move_index)
    moves_to_remove.sort(reverse=True)
    for index in moves_to_remove:
        del valid_moves_indexed[index]

    sorted_moves = find_local_max_moves(valid_moves_indexed)

    final_moves = [move[0] for move in sorted_moves]

    return final_moves

def find_local_max_moves(valid_moves_indexed):
    """
    Find and filter the local maximum moves within the valid moves.

    Args:
        valid_moves_indexed (list): List of indexed valid moves.

    Returns:
        list: List of local maximum moves within the valid moves.
    """
    sorted_moves = []

    if len(valid_moves_indexed) > 1:
        current_max_index = 0
        starting_range_index = valid_moves_indexed[0][1]
        current_max_val = valid_moves_indexed[0][2]

        for i in range(len(valid_moves_indexed)):
            buffer = apply_buffer(0, 0)  # Reset buffer for each iteration

            if valid_moves_indexed[i][1] > starting_range_index + 3:
                sorted_moves.append(valid_moves_indexed[current_max_index])
                starting_range_index = valid_moves_indexed[i][1]
                current_max_val = valid_moves_indexed[i][2]
                current_max_index = i
            else:
                if valid_moves_indexed[i][2] > current_max_val:
                    current
