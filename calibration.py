# import ctypes
# from atc3dg_functions import initialize_tracker, tare_sensor, get_sensor_data

# # Load the tracker system DLL
# tracker_lib = ctypes.CDLL('ATC3DG64.DLL')

# # Function to initialize the tracker system using the DLL
# def initialize_tracker_system():
#     result = tracker_lib.InitializeSystem()
#     if result == 0:
#         print("Tracker system initialized successfully.")
#     else:
#         print("Failed to initialize the tracker system.")
#         return False
#     return True

# # Function to tare (calibrate) the sensor
# def tare_sensor_system(sensor_id):
#     result = tracker_lib.TareSensor(sensor_id)  # Assuming TareSensor function is in DLL
#     if result == 0:
#         print(f"Sensor {sensor_id} successfully tared (calibrated).")
#     else:
#         print(f"Failed to tare sensor {sensor_id}.")
#         return False
#     return True

# # Function to retrieve sensor data after calibration
# def get_calibrated_sensor_data(sensor_id):
#     sensor_data = get_sensor_data(sensor_id)  # Assuming get_sensor_data is defined in atc3dg_functions.py
#     print(f"Sensor {sensor_id} Data Post-Tare: {sensor_data}")
#     return sensor_data

# # Main function to run the calibration
# def calibrate_tracker():
#     # Step 1: Initialize the tracker system
#     if not initialize_tracker_system():
#         return

#     # Step 2: Tare (Calibrate) the sensor
#     sensor_id = 1  # Example: Calibrating sensor 1
#     if not tare_sensor_system(sensor_id):
#         return

#     # Step 3: Verify by retrieving sensor data
#     get_calibrated_sensor_data(sensor_id)

# if __name__ == "__main__":
#     calibrate_tracker()
from Tracker_system import Tracker_system
import logging
import time

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def calibrate_tracker():
    try:
        # Step 1: Initialize the tracker system
        tracker_system = Tracker_system()
        tracker_system.initialize_tracker()
        logging.info("Tracker system initialized successfully.")
        time.sleep(1)
        # Step 2: Align the sensor angles
        tracker_system.align_angles()
        logging.info("Sensor angles aligned successfully.")

        # Step 3: Tare the sensor (reset the timer)
        tracker_system.reset_timer()
        logging.info("Sensor tared to zero position and timer reset.")

        # Step 4: Verify calibration by collecting current sensor data
        current_position = tracker_system.collect_current()
        logging.info(f"Calibration complete. Current Position and Angles: {current_position}")

    except Exception as e:
        logging.error(f"Calibration failed: {e}")

if __name__ == "__main__":
    calibrate_tracker()
