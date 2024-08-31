from __future__ import absolute_import, division, print_function
from builtins import *  # @UnusedWildImport

from time import sleep
import ctypes

from mcculw import ul
from mcculw.enums import ScanOptions, InterfaceType
from mcculw.device_info import DaqDeviceInfo

try:
    from console_examples_util import config_first_detected_device
except ImportError:
    from DAQ_examples.console.console_examples_util import config_first_detected_device

class Load_Cell:
    def __init__(self):
        self.board_num = None
        self.daq_dev_info = None
        self.low_channel = None
        self.high_channel = None
        self.num_channels = None
        self.rate = None
        self.ai_info = None
        self.ai_range = None
        self.scan_options = None
        self.memhandle = None
        self.points_per_channel = None


    def initialize_daq(self):
        self.board_num = 0
        ul.ignore_instacal()
        devices = ul.get_daq_device_inventory(InterfaceType.ANY)
        device = devices[0]
        ul.create_daq_device(self.board_num, device)
        self.daq_dev_info = DaqDeviceInfo(self.board_num)
        ul.a_input_mode(self.board_num, 1)  # This sets board to single-ended mode collection

        if not self.daq_dev_info.supports_analog_input:
            raise Exception('Error: The DAQ device does not support '
                            'analog input')

        # Define channels (0 for anterior, 2 for inferior, 4 for posterior)
        self.low_channel = 0
        self.high_channel = 4
        self.num_channels = self.high_channel - self.low_channel + 1

        # Define sampling rate per channel
        self.rate = 100  # Hz

        self.ai_info = self.daq_dev_info.get_ai_info()
        self.ai_range = self.ai_info.supported_ranges[0]
        self.scan_options = ScanOptions.FOREGROUND
        self.points_per_channel = 100

    def stop_reading(self):
        # ul.stop_background(self.board_num, FunctionType.AIFUNCTION)  # don't need this because I'm using FOREGROUND
        if self.memhandle:
            # Free the buffer in a finally block to prevent  a memory leak.
            ul.win_buf_free(self.memhandle)
        # ul.release_daq_device(self.board_num)  # I don't release the board so I can keep using it

    def get_continuous_reading(self):

        # Define arrays to hold data
        anterior_data = []
        inferior_data = []
        posterior_data = []

        total_count = self.points_per_channel * self.num_channels

        # Use the win_buf_alloc method for devices with a resolution <= 16
        self.memhandle = ul.win_buf_alloc(total_count)
        # Convert the memhandle to a ctypes array.
        ctypes_array = ctypes.cast(self.memhandle, ctypes.POINTER(ctypes.c_ushort))

        # Check if the buffer was successfully allocated
        if not self.memhandle:
            raise Exception('Error: Failed to allocate memory')

        # Start the scan
        ul.a_in_scan(self.board_num, self.low_channel, self.high_channel, total_count, self.rate, self.ai_range, self.memhandle, self.scan_options)

        data_index = 0
        for index in range(self.points_per_channel):
            # display_data = [index]
            for tag in range(self.num_channels):
                eng_value = ul.to_eng_units(self.board_num, self.ai_range, ctypes_array[data_index])
                # display_data.append('{:.5f}'.format(eng_value))
                if tag == 0:
                    anterior_data.append(float('{:.5f}'.format(eng_value)))
                elif tag == 2:
                    inferior_data.append(float('{:.5f}'.format(eng_value)))
                elif tag == 4:
                    posterior_data.append(float('{:.5f}'.format(eng_value)))
                data_index += 1


            # Wait before adding more values to the display.
            # sleep(10/1000)  # same as EM tracker

            # status, curr_count, curr_index = ul.get_status(self.board_num, FunctionType.AIFUNCTION)

        self.stop_reading()

        return anterior_data, posterior_data
