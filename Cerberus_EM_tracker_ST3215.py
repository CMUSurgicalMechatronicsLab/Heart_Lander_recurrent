'''
Author: Yi Wang
Date: Aug-2024
'''

from EM_tracker.Tracker_system import *
from EM_tracker.atc3dg_types import *
import numpy as np
from math import *
from ST3215 import ST3215  # Import the ST3215 class for motor control


def main():
    CerbRecord = DOUBLE_POSITION_ANGLES()

    Cerb_injector = Tracker_system()

    # Initialize motors
    anterior_motor = ST3215(port='COM1', baudrate=115200)
    inferior_motor = ST3215(port='COM2', baudrate=115200)
    posterior_motor = ST3215(port='COM3', baudrate=115200)
    anterior_motor.enable()
    inferior_motor.enable()
    posterior_motor.enable()

    # Set delay in ms
    Cerb_injector.set_delay(1000)

    # Initialize all tracker components
    print("Initializing all Tracker components...")
    Cerb_injector.initialize_tracker()
    print("Done Initializing. Begin Collecting...")

    # Single point collection test
    print("Now collecting single point:")
    CerbRecord = Cerb_injector.collect_current()
    print("x:", CerbRecord[0], "y:", CerbRecord[1], "z:", CerbRecord[2], "r:", CerbRecord[3],
          "e:", CerbRecord[4], "a:", CerbRecord[5])

    # Motor Control Example (if needed)
    anterior_motor.set_angle(90)  # Example command to move the anterior motor

    # Disable motors and close the serial connections
    anterior_motor.disable()
    inferior_motor.disable()
    posterior_motor.disable()
    anterior_motor.close()
    inferior_motor.close()
    posterior_motor.close()

    # Continuous points collection test (optional)
    # print("Now collecting continuous points:")
    # CerbRecords = Cerb_injector.collect_continuous()
    # Processing the collected points as needed


if __name__ == '__main__':
    main()
