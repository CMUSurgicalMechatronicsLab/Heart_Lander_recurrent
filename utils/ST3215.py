'''
Author: Yi Wang
Date: Aug-2024
'''
import serial  # PySerial for serial communication with ESP32
import time


class ST3215:
    """ Class for controlling the ST3215 servo motor via ESP32 """

    def __init__(self, port='COM6', baudrate=115200, timeout=1):
        """
        Initialize the serial connection to the ESP32.
        :param port: The COM port to use (e.g., 'COM6' for Windows or '/dev/ttyUSB0' for Linux).
        :param baudrate: The baud rate for the serial communication.
        :param timeout: Read timeout for serial communication.
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Give some time for the serial connection to establish

    def send_command(self, command):
        """
        Send a command to the ESP32.
        :param command: Command string to send.
        """
        command += '\n'  # Ensure the command is terminated with a newline
        self.ser.write(command.encode())
        time.sleep(0.1)  # Short delay to ensure command is processed

    def read_response(self):
        """
        Read the response from the ESP32.
        :return: The response string from the ESP32.
        """
        response = self.ser.readline().decode().strip()
        return response

    def set_angle(self, angle):
        """
        Set the servo to a specific angle.
        :param angle: The target angle (in degrees) to set the servo to.
        """
        command = f'SET_ANGLE {angle}'
        self.send_command(command)
        response = self.read_response()
        print(response)

    def get_angle(self):
        """
        Get the current angle of the servo.
        :return: The current angle of the servo.
        """
        command = 'GET_ANGLE'
        self.send_command(command)
        response = self.read_response()
        print(f"Current angle: {response} degrees")
        return float(response)

    def set_speed(self, speed):
        """
        Set the speed of the servo.
        :param speed: The speed value to set (specific to the motor's capabilities).
        """
        command = f'SET_SPEED {speed}'
        self.send_command(command)
        response = self.read_response()
        print(response)

    def disable(self):
        """
        Disable the servo (turn off PWM signal).
        """
        command = 'DISABLE'
        self.send_command(command)
        response = self.read_response()
        print("Servo disabled.")

    def enable(self):
        """
        Enable the servo (turn on PWM signal).
        """
        command = 'ENABLE'
        self.send_command(command)
        response = self.read_response()
        print("Servo enabled.")

    def close(self):
        """
        Close the serial connection.
        """
        self.ser.close()
        print("Serial connection closed.")


if __name__ == '__main__':
    servo = ST3215(port='COM6', baudrate=115200)  # Adjust the port to your setup

    servo.enable()
    servo.set_angle(90)  # Move the servo to 90 degrees
    time.sleep(2)
    current_angle = servo.get_angle()  # Read the current angle
    servo.set_speed(50)  # Set the servo speed (depends on your servo's capabilities)
    servo.set_angle(180)  # Move the servo to 180 degrees
    time.sleep(2)
    servo.disable()  # Disable the servo

    servo.close()  # Close the serial connection
