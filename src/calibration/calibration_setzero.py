from Tracker_system import Tracker_system
import logging
import time

# Configure logging for debugging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class ManualTare:
    def __init__(self):
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.is_tared = False

    def calibrate_tracker(self):
        try:
            # Step 1: Initialize the tracker system
            tracker_system = Tracker_system()

            # Debugging output to verify sensor initialization
            logging.info("Attempting to initialize the tracker system...")

            # Call the initialization method
            tracker_system.initialize_tracker()
            logging.info("Tracker system initialized successfully.")

            # Add a small delay to ensure the system is fully initialized
            time.sleep(1)

            # Step 2: Align the sensor angles
            tracker_system.align_angles()
            logging.info("Sensor angles aligned successfully.")

            # Add a small delay before taring
            time.sleep(1)

            # Step 3: Perform manual taring (collect and store the current position as the offset)
            self.manual_tare(tracker_system)

            # Step 4: Collect sensor data after taring
            current_position = tracker_system.collect_current()
            corrected_position = self.apply_tare(current_position)

            if corrected_position:
                logging.info(f"Calibration complete. Corrected Position and Angles: {corrected_position}")
            else:
                logging.error("No valid sensor data collected. Check the sensor connections and configuration.")

        except Exception as e:
            logging.error(f"Calibration failed due to an error: {e}")

    def manual_tare(self, tracker_system):
        """
        Capture the current position and store it as the offset.
        """
        try:
            # Collect the initial position (before taring)
            current_position = tracker_system.collect_current()

            if current_position:
                # Store the initial position as the offset
                self.offset_x = current_position[0]
                self.offset_y = current_position[1]
                self.offset_z = current_position[2]
                self.is_tared = True

                logging.info(f"Manual tare applied with offsets: x={self.offset_x}, y={self.offset_y}, z={self.offset_z}")
            else:
                logging.error("Failed to collect current position for manual taring.")

        except Exception as e:
            logging.error(f"Manual taring failed: {e}")

    def apply_tare(self, current_position):
        """
        Apply the stored offset to the current position to produce a zeroed result.
        """
        if self.is_tared and current_position:
            # Subtract the offset from the current position
            corrected_x = current_position[0] - self.offset_x
            corrected_y = current_position[1] - self.offset_y
            corrected_z = current_position[2] - self.offset_z

            return [corrected_x, corrected_y, corrected_z] + current_position[3:]  # Return corrected position and original angles
        else:
            logging.error("Manual tare not applied yet or invalid sensor data.")
            return None


if __name__ == "__main__":
    tare_system = ManualTare()
    tare_system.calibrate_tracker()