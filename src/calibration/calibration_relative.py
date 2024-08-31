
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
