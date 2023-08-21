"""
old_spike_detect.py - Module for spike detection and move validation.

This module contains functions to detect valid moves in the sensor data sequence,
perform spike detection, and process the sequence to generate the final sequence
of detected moves.

In legacy folder because it is a god function. Documentation is made
in the README.md for a higher level explanation of these algorithms.

"""

sensitivity = 4
buffer_offset = 4 # there are typically 3 elements of feedback
                  # for example forward move is [20, -20, -18, -12, -4, 0, 0, 0]
z_offset = 10 # don't use z_offset on raw data (flip z needs to be not around 0 to detect flips as the sign of the number)


# --------------------------------------------------------------------------------------------------------------------------------------------
# IMU DATA PARSING -> SEQUENCE
# --------------------------------------------------------------------------------------------------------------------------------------------

# Return a list of all the valid moves that happened in the sequence
# Note sequence is a dict of <string, list> where all lists are the same length
def check_sequence(sequence):
    buffer = 0


    # List of pairs (index, move)
    valid_moves_indexed = []

    # Check that the imu was flipped over at some
    started_up = False

    for i, z in enumerate(sequence["AZ"]):

        # Update buffer
        if buffer < 0:
            buffer = 0
        elif buffer > 0:
            buffer = buffer - 1

        if buffer == 0:
            if z >= 0:
                started_up = True
            elif z < 0 and started_up:
                # To avoid noise, check that neighbouring values are also negative (for sure flipped for a period of time)
                if (i + 1) < (len(sequence["AZ"]) - 1) and (i - 1) > 0:
                    flip = True
                    # If the IMU is rotated up in the neighbouring data, ignore the data
                    for j in range(i - 1, i + 1 + 1):
                        if sequence["AZ"][j] > 0:
                            flip = False
                    if flip == True:
                        valid_moves_indexed.append(("FLIP", i, 0))
                        buffer = buffer + buffer_offset
                        started_up = False

    # X: check move forward and ignore move backward
    # To deal with inverse acceleration feedback, add a buffer whenever the IMU is moved backward
    # For example, moving backward will spike -sensitivitym/s^2 back then sensitivitym/s^2 forward withing a short period after
    # We need to ignore that sensitivitym/s^2 signal since it will detect as forward motion (when really we moved the IMU backward)
    # We achieve this by ignoring the following elements after a negative motion
    for i, x in enumerate(sequence["AX"]):
        # Update buffer
        if buffer < 0:
            buffer = 0
        elif buffer > 0:
            buffer = buffer - 1

        if buffer == 0:
            # sensitivity of a false signal is lower than a correct signal
            # this is because we want to ignore more noise even if that means missing some correct signals
            # for debugging, it's easier to have a small amount of correct signals
            # than having many all the correct signals and a lot of bad signals
            if x < (-1 * sensitivity):
                # ignore the next buffer_offset elements in list
                buffer = buffer + buffer_offset
            elif x > sensitivity :
                valid_moves_indexed.append(("RIGHT", i, x))
                buffer = buffer + buffer_offset
                started_up = False

    # Y
    for i, y in enumerate(sequence["AY"]):
        # Update buffer
        if buffer < 0:
            buffer = 0
        elif buffer > 0:
            buffer = buffer - 1

        if buffer == 0:
            if y < (-1 * sensitivity):
                # ignore the next buffer_offset elements in list
                buffer = buffer + buffer_offset
            elif y > sensitivity:
                valid_moves_indexed.append(("FORWARD", i, y))
                buffer = buffer + buffer_offset

    # Z
    for i, z in enumerate(sequence["AZ"]):
        # Update buffer
        if buffer < 0:
            buffer = 0
        elif buffer > 0:
            buffer = buffer - 1

        # Note: z axis offsetting requires 2 cases
        # if z > 0 then SUBTRACT 9.8m/s^s
        # if z < 0 then ADD 9.8m/s^s
        if buffer == 0:
            # if see a negative acceleration motion first, not +Z motion
            if z - z_offset < (-1 * sensitivity):
                # ignore the next buffer_offset elements in list
                buffer = buffer + buffer_offset
            elif z - z_offset > sensitivity:
                valid_moves_indexed.append(("UP", i, (z - z_offset)))
                buffer = buffer + buffer_offset

    # -X
    for i, x in enumerate(sequence["AX"]):
        # Update buffer
        if buffer < 0:
            buffer = 0
        elif buffer > 0:
            buffer = buffer - 1

        if buffer == 0:
            if x > sensitivity:
                # ignore the next buffer_offset elements in list
                buffer = buffer + buffer_offset
            elif x < -1 * sensitivity :
                valid_moves_indexed.append(("LEFT", i, -1 * x))
                buffer = buffer + buffer_offset

    # -Y
    for i, y in enumerate(sequence["AY"]):
        # Update buffer
        if buffer < 0:
            buffer = 0
        elif buffer > 0:
            buffer = buffer - 1

        if buffer == 0:
            if y > sensitivity:
                # ignore the next buffer_offset elements in list
                buffer = buffer + buffer_offset
            elif y < -1 * sensitivity:
                valid_moves_indexed.append(("BACKWARD", i, -1 * y))
                buffer = buffer + buffer_offset

    # -Z
    for i, z in enumerate(sequence["AZ"]):
        # Update buffer
        if buffer < 0:
            buffer = 0
        elif buffer > 0:
            buffer = buffer - 1

        if buffer == 0:
            # if see a negative acceleration motion first, not +Z motion
            if (z - z_offset) > (sensitivity):
                # ignore the next buffer_offset elements in list
                buffer = buffer + buffer_offset
            elif (z - z_offset) < (-1 * sensitivity):
                valid_moves_indexed.append(("DOWN", i, (z - z_offset) * -1))
                buffer = buffer + buffer_offset

    # --------------------------------------------------------------------------------------------------------------------------------------------
    # SEQUENCE PROCESSING
    # --------------------------------------------------------------------------------------------------------------------------------------------
    if len(valid_moves_indexed) == 0:
        return valid_moves_indexed
    # Processing sequence
    # Sort based on time -> filter moves caused by feedback -> put in list of moves

    # Sort the valid moves based on index (which is time in 0.1s) ----------------------------------------------------------
    print("\n\nunsorted:", valid_moves_indexed, "\n\n")
    valid_moves_indexed.sort(key = lambda x: x[1])
    print("sorted", valid_moves_indexed, "\n\n")

    # We now have to filter the overlapping moves that shouldn't exist
    # This depends on a precedence
    # For example, FLIP can also triggers other moves if the flip is aggressive
    # We do this be removing any neighbouring moves that occured within +/-0.3ms of the flip
    tolerance = 4

    # Note the filters occur in order based on precedence
    # Flip, up/down, left/right (these compound)

    # Filter so that flip takes precedence over all moves ----------------------------------------------------------
    print("flip filter begin")
    flip_occurence_indices = []
    for pair in valid_moves_indexed:
        if pair[0] == "FLIP":
            flip_occurence_indices.append(pair[1])

    # Remove moves from list based on filter
    moves_to_remove = []
    for index in flip_occurence_indices:
        for move_index, pair in enumerate(valid_moves_indexed):
            # first condition checks if move is within tolerance * 0.1ms of the flip
            if abs(pair[1] - index) < tolerance  and pair[0] != "FLIP":
                # print("try to remove", pair, "at move index", move_index)
                # print("removing", pair)
                moves_to_remove.append(move_index)

    moves_to_remove.sort(reverse=True)  # Remove elements from end of list to prevent index errors
    for index in moves_to_remove:
        print("\tremoving", valid_moves_indexed[index])
        del valid_moves_indexed[index]

    print("flip filtered:", valid_moves_indexed, "\n\n")

    print("local max filter begin")
    sorted_moves = []
    if len(valid_moves_indexed) > 1:
        '''
        valid_moves_indexed[0][0] move string
        valid_moves_indexed[0][1] time occurrence value of move
        valid_moves_indexed[0][2] strength value of move
        '''
        i = 0   # current index
        starting_range_index = valid_moves_indexed[0][1]    # index for neighbour comparison
        current_max_val = valid_moves_indexed[0][2]         # value of max between neighbours
        current_max_index = 0                               # index of max neighbour in valid_moves_indexed
        while i < len(valid_moves_indexed):
            # outside neighbour compare region, update new starting comparison element
            if valid_moves_indexed[i][1] > starting_range_index + 3:
                # add the previous max to sorted_moves
                print("Appending Max", valid_moves_indexed[current_max_index])
                sorted_moves.append(valid_moves_indexed[current_max_index])
                # update variables to find next max
                starting_range_index = valid_moves_indexed[i][1]   
                current_max_val = valid_moves_indexed[i][2]       
                current_max_index = i                      
            # inside neighbour compare region, compare and update current max element
            else:
                if valid_moves_indexed[i][2] > current_max_val:
                    current_max_val = valid_moves_indexed[i][2]
                    current_max_index = i  
            # traverse
            i = i + 1
        
        print("Appending Max", valid_moves_indexed[current_max_index])
        sorted_moves.append(valid_moves_indexed[current_max_index])
    else:
        sorted_moves.append(valid_moves_indexed[0])
        
    print("final sequence:", sorted_moves)
    # print()
    final_moves = []
    for move in sorted_moves:
        final_moves.append(move[0])
    print("\n")

    return final_moves