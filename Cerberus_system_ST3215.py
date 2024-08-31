'''
Author:YI wang
Date: Aug-2024
'''
import ctypes
from time import time, sleep
import copy
import numpy as np
from math import *

import EM_tracker.atc3dg_functions as api
from EM_tracker.atc3dg_types import *

from ST3215 import ST3215  # Import your new motor class

from mcculw import ul
from mcculw.enums import ScanOptions, InterfaceType
from mcculw.device_info import DaqDeviceInfo

try:
    from console_examples_util import config_first_detected_device
except ImportError:
    from DAQ_examples.console.console_examples_util import config_first_detected_device


class Cerberus_system:
    def __init__(self):
        # EM Tracker parameters
        self.record = DOUBLE_POSITION_ANGLES()
        self.recordPtr = ctypes.pointer(self.record)
        self.system_configuration = None
        self.attached_sensors = None
        self.init_time = None
        self._is_init = False
        self.sensorID = None
        self.delay = None

        # ST3215 Motors parameters
        self.anterior_motor = None
        self.inferior_motor = None
        self.posterior_motor = None

        # Load Cells parameters
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

    def __del__(self):
        self.turn_off_tracker(ignore_error=True)

    def initialize_tracker(self):
        """Initialize EM tracker"""

        if self.is_init:
            return

        error_code = api.InitializeBIRDSystem()
        if error_code != 0:
            self._error_handler(error_code)

        self.sensorID = ctypes.c_ushort(0)
        api.SetSystemParameter(api.SystemParameterType.SELECT_TRANSMITTER,
                               ctypes.pointer(self.sensorID), 2)

        # Measurement properties
        measurement_rate = 80  # Hz
        metric = True  # True for mm, False for inch
        max_range = 36  # inches
        self.system_configurations(measurement_rate, metric, max_range, power_line=60, report_rate=1)

        # Align orientation of EM sensor (calibration step before introducer mechanisms)
        self.align_angles()
        AnglesAlignRecord = DOUBLE_ANGLES()
        pAnglesAlignRecord = ctypes.pointer(AnglesAlignRecord)
        error_code = api.GetSensorParameter(self.sensorID, api.SensorParameterType.ANGLE_ALIGN, pAnglesAlignRecord, ctypes.sizeof(AnglesAlignRecord))
        if error_code != 0:
            self._error_handler(error_code)
        print("Aligned angles to: ")
        print("roll:", AnglesAlignRecord.r, "elevation:", AnglesAlignRecord.e, "azimuth:", AnglesAlignRecord.a)

        self.reset_timer()
        self._is_init = True

    def system_configurations(self, measurement_rate, metric, max_range, power_line=60, report_rate=1):
        """read and set system and EM sensor configuration"""

        # system configuration
        sysconf = api.SYSTEM_CONFIGURATION()
        psys_conf = ctypes.pointer(sysconf)

        # report SystemConfiguration (default)
        error_code = api.GetBIRDSystemConfiguration(psys_conf)
        if error_code != 0:
            self._error_handler(error_code)
        self.system_configuration = sysconf

        # read attached sensors config
        sensor_conf = api.SENSOR_CONFIGURATION()
        psensor_conf = ctypes.pointer(sensor_conf)
        attached_sensors = []
        for cnt in range(self.system_configuration.numberSensors):
            error_code = api.GetSensorConfiguration(ctypes.c_ushort(cnt),
                                                    psensor_conf)
            if error_code != 0:
                self._error_handler(error_code)
            elif sensor_conf.attached:
                attached_sensors.append(cnt + 1)
        self.attached_sensors = attached_sensors

        # Set system parameters
        mR = ctypes.c_double(measurement_rate)
        max_range = ctypes.c_double(max_range)
        metric = ctypes.c_int(int(metric))
        power_line = ctypes.c_double(power_line)
        report_rate = ctypes.c_ushort(report_rate)

        error_code = api.SetSystemParameter(
            api.SystemParameterType.MEASUREMENT_RATE,
            ctypes.pointer(mR), 8)
        if error_code != 0:
            self._error_handler(error_code)

        error_code = api.SetSystemParameter(
            api.SystemParameterType.MAXIMUM_RANGE,
            ctypes.pointer(max_range), 8)
        if error_code != 0:
            self._error_handler(error_code)

        error_code = api.SetSystemParameter(api.SystemParameterType.METRIC,
                                            ctypes.pointer(metric), 4)
        if error_code != 0:
            self._error_handler(error_code)

        error_code = api.SetSystemParameter(
            api.SystemParameterType.POWER_LINE_FREQUENCY,
            ctypes.pointer(power_line), 8)
        if error_code != 0:
            self._error_handler(error_code)

        error_code = api.SetSystemParameter(api.SystemParameterType.REPORT_RATE,
                                            ctypes.pointer(report_rate), 2)
        if error_code != 0:
            self._error_handler(error_code)

        # report SystemConfiguration (custom settings)
        error_code = api.GetBIRDSystemConfiguration(psys_conf)
        if error_code != 0:
            self._error_handler(error_code)
        self.system_configuration = sysconf

    def set_delay(self, delay):
        """set EM tracker delay between collections"""
        self.delay = delay

    def reset_timer(self):
        self.init_time = time()

    def align_angles(self):
        """
        Align the angles of the EM sensor in the injector head. Perform this step
        with the injector head placed directly in front of the front hemisphere of the transmitter.
        """

        CerbRecord = DOUBLE_POSITION_ANGLES()

        zangle_offset = 0

        while True:
            CerbRecord = self.collect_current()
            zangle_offset = CerbRecord[5]
            print("z-angle:", zangle_offset)
            if zangle_offset < 91.0 and zangle_offset > 89.0:
                xangle_offset = CerbRecord[3]
                yangle_offset = CerbRecord[4]
                break

        xangle_offset = ctypes.c_double(xangle_offset)
        yangle_offset = ctypes.c_double(yangle_offset)
        zangle_offset = ctypes.c_double(zangle_offset)

        AnglesAlign = DOUBLE_ANGLES()
        AnglesAlign.r = xangle_offset
        AnglesAlign.e = yangle_offset
        AnglesAlign.a = zangle_offset
        pAnglesAlign = ctypes.pointer(AnglesAlign)

        api.SetSensorParameter(self.sensorID, api.SensorParameterType.ANGLE_ALIGN, pAnglesAlign, ctypes.sizeof(AnglesAlign))

    @property
    def is_init(self):
        """Returns True if EM Trakstar is initialized"""
        return self._is_init

    def _error_handler(self, error_code):
        """error handler for EM tracker"""
        print("** Error: ", error_code)
        txt = " " * 500
        pbuffer = ctypes.c_char_p(txt)
        api.GetErrorText(error_code, pbuffer, ctypes.c_int(500),
                         api.MessageType.VERBOSE_MESSAGE)
        print(pbuffer.value)
        self.turn_off_tracker(ignore_error=True)
        raise RuntimeError("** trakSTAR Error")

    def turn_off_tracker(self, ignore_error=True):
        """for turning off the EM tracker"""
        if not self.is_init:
            return
        self.attached_sensors = None
        self.system_configuration = None
        error_code = api.CloseBIRDSystem()
        if error_code != 0 and not ignore_error:
            self._error_handler(error_code)
        self._is_init = False

    def collect_current(self):
        """collect a single data point from the EM tracker"""
        for i in range(len(self.attached_sensors)):
            status = api.GetSensorStatus(self.sensorID)
            if status == 0:
                error_code = api.GetSynchronousRecord(self.sensorID, self.recordPtr, ctypes.sizeof(self.record))
                if error_code != 0:
                    self._error_handler(error_code)
        x = copy.copy(self.record.x)
        y = copy.copy(self.record.y)
        z = copy.copy(self.record.z)
        r = copy.copy(self.record.r)
        e = copy.copy(self.record.e)
        a = copy.copy(self.record.a)

        return [x, y, z, r, e, a]

    def collect_continuous(self):
        """collect continuous data points from the EM tracker"""
        recorded = 0
        records = []
        try:
            while True:
                records.append(self.collect_current())
                sleep(self.delay/1000)  # convert to seconds
                print("x:", records[recorded][0], " y:", records[recorded][1], " z:", records[recorded][2], " roll:", records[recorded][3], " elevation:", records[recorded][4], " azimuth:", records[recorded][5])
                recorded += 1
        except KeyboardInterrupt: # Use Ctrl-c to end loop
            print("Tracked turned off")
            pass

        self.turn_off_tracker()
        return records

    def initialize_motors(self):
        """Initialize the ST3215 motors via the ESP32."""
        # Initialize 3 ST3215 motor instances
        self.anterior_motor = ST3215(port='COM6', baudrate=115200)
        self.inferior_motor = ST3215(port='COM6', baudrate=115200)
        self.posterior_motor = ST3215(port='COM6', baudrate=115200)

        # Enable the motors
        self.anterior_motor.enable()
        self.inferior_motor.enable()
        self.posterior_motor.enable()

        print("ST3215 motors initialized.")

    def initialize_daq(self):
        """Initialize DAQ (USB-1208LS) for reading voltages from load cells"""

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
        self.rate = 200  # Hz

        self.ai_info = self.daq_dev_info.get_ai_info()
        self.ai_range = self.ai_info.supported_ranges[0]
        self.scan_options = ScanOptions.FOREGROUND
        self.points_per_channel = 1

    def homing(self, time_duration):
        """first step to pull robot components towards tabletop setup if not already. You define time"""
        self.anterior_motor.set_angle(0)
        self.inferior_motor.set_angle(0)
        self.posterior_motor.set_angle(0)
        sleep(time_duration)

    def introducer_mech_posterior_slack(self):
        """giving slack to posterior base for positioning"""
        self.posterior_motor.set_angle(90)  # Assume 90 degrees is slack
        sleep(35)
        self.posterior_motor.set_angle(0)

    def introducer_mech_anterior_inferior_slack(self):
        """giving slack to anterior and inferior to bring them close to subxiphoid port"""
        self.anterior_motor.set_angle(90)
        self.inferior_motor.set_angle(90)
        sleep(25)
        self.anterior_motor.set_angle(0)
        self.inferior_motor.set_angle(0)

    def introducer_mech_anterior_slack(self):
        """giving slack to anterior base for positioning"""
        self.anterior_motor.set_angle(120)  # Assume 120 degrees is more slack
        sleep(6)
        self.anterior_motor.set_angle(0)

    def introducer_mech_inferior_slack(self):
        """giving slack to inferior base for positioning"""
        self.inferior_motor.set_angle(120)
        sleep(1)
        self.inferior_motor.set_angle(0)

    def get_voltage_reading(self):
        """
        Collect voltage reading from DAQ device. I only have it collecting for the anterior and posterior
        right now until we get an inferior load cell.
        """
        anterior_data = 0  # channel 0
        posterior_data = 0  # channel 4

        channel_list = [0,4]

        for channel in channel_list:
            value = ul.v_in(self.board_num, channel, self.ai_range)
            value = np.round(value, 3)
            if channel == 0:
                anterior_data = value
            elif channel == 4:
                posterior_data = value

        return anterior_data, posterior_data

    def stop_reading(self):
        """To stop reading voltages from the DAQ"""
        if self.memhandle:
            ul.win_buf_free(self.memhandle)

    def inferior_base(self):
        """Used to determine the position of the inferior base after positioning on the heart"""

        recorded = 0
        records = []

        # Define offset distances determined from CAD model (if I do angle align). We are in front of transmitter's
        # front hemisphere
        x0 = 0
        y0 = -11.76991058
        z0 = 4.11660638

        # Collect points to warm up EM sensor
        timeout = time() + 3
        while True:
            tmp = self.collect_current()
            if time() > timeout:
                break

        while recorded < 101:  # collect 100 hundred points
            records.append(self.collect_current())
            sleep(self.delay / 1000)  # convert to seconds
            recorded += 1

        inferior_points = np.empty([len(records), 15])  # initialize numpy array for data

        for i in range(len(records)):
            inferior_points[i, 0] = records[i][0]  # x
            inferior_points[i, 1] = records[i][1]  # y
            inferior_points[i, 2] = records[i][2]  # z

            # Convert Euler Angles from degrees to radians
            r = 0.0174533 * records[i][3]
            e = 0.0174533 * records[i][4]
            a = 0.0174533 * records[i][5]

            # Calculate Rotation Matrix from Euler Angles. I have to do this b/c their matrix and quaternions structures don't work
            m11 = cos(e) * cos(a)
            m12 = cos(e) * sin(a)
            m13 = -sin(e)
            m21 = -(cos(r) * sin(a)) + (sin(r) * sin(e) * cos(a))
            m22 = (cos(r) * cos(a)) + (sin(r) * sin(e) * sin(a))
            m23 = sin(r) * cos(e)
            m31 = (sin(r) * sin(a)) + (cos(r) * sin(e) * cos(a))
            m32 = -(sin(r) * cos(a)) + (cos(r) * sin(e) * sin(a))
            m33 = cos(r) * cos(e)

            inferior_points[i, 3] = m11
            inferior_points[i, 4] = m12
            inferior_points[i, 5] = m13
            inferior_points[i, 6] = m21
            inferior_points[i, 7] = m22
            inferior_points[i, 8] = m23
            inferior_points[i, 9] = m31
            inferior_points[i, 10] = m32
            inferior_points[i, 11] = m33
            inferior_points[i, 12] = records[i][3]
            inferior_points[i, 13] = records[i][4]
            inferior_points[i, 14] = records[i][5]

        avg_vals = np.mean(inferior_points, axis=0)
        xsens = avg_vals[0]
        ysens = avg_vals[1]
        zsens = avg_vals[2]
        print("xsens_inferior:", xsens)
        print("ysens_inferior:", ysens)
        print("zsens_inferior:", zsens)

        m11sens = avg_vals[3]
        m12sens = avg_vals[4]
        m13sens = avg_vals[5]
        m21sens = avg_vals[6]
        m22sens = avg_vals[7]
        m23sens = avg_vals[8]
        m31sens = avg_vals[9]
        m32sens = avg_vals[10]
        m33sens = avg_vals[11]

        rsens = avg_vals[12]
        esens = avg_vals[13]
        asens = avg_vals[14]

        x_inf = xsens + x0 * m11sens + y0 * m21sens + z0 * m31sens
        y_inf = ysens + x0 * m12sens + y0 * m22sens + z0 * m32sens
        z_inf = zsens + x0 * m13sens + y0 * m23sens + z0 * m33sens

        transformed_records = self.injector_head_transform(records)

        return [x_inf, y_inf, z_inf, rsens, esens, asens], records

    def collect_trajectory(self):
        """
        Used to collect trajectory points on the bounds of the triangular workspace required for registration
        """
        Cerberus_reg_records = []  # registration points for transformation

        base_anterior_flag = 0
        base_posterior_flag = 0

        anterior_tension_threshold = 0.3  # determined through experimentation
        posterior_tension_threshold = 0.3  # determined through experimentation

        while True:  # Determine anterior base position
            self.anterior_motor.set_angle(75)  # Example value for pulling cable
            anterior_tension, _ = self.get_voltage_reading()

            Cerberus_reg_records.append(self.collect_current())

            if anterior_tension >= anterior_tension_threshold and base_anterior_flag == 0:
                print("\nReached anterior base")
                anterior_position, Cerberus_reg_anterior_records_transform = self.anterior_base()
                self.anterior_motor.set_angle(0)  # Stop rotating motor
                base_anterior_flag = 1
                break

        sleep(2)
        self.anterior_motor.set_angle(130)  # Example value for releasing cable
        sleep(3)
        self.anterior_motor.set_angle(0)

        while True:  # Determine posterior base position
            self.posterior_motor.set_angle(75)  # Example value for pulling cable
            _, posterior_tension = self.get_voltage_reading()

            Cerberus_reg_records.append(self.collect_current())

            if posterior_tension >= posterior_tension_threshold and base_posterior_flag == 0:
                print("\nReached posterior base")
                posterior_position, Cerberus_reg_posterior_records_transform = self.posterior_base()
                self.posterior_motor.set_angle(0)  # Stop rotating motor
                base_posterior_flag = 1
                break

        sleep(2)
        self.posterior_motor.set_angle(130)  # Example value for releasing cable
        sleep(3)
        self.posterior_motor.set_angle(0)

        self.inferior_motor.set_angle(75)  # Example value for pulling cable
        timeout = time() + 30
        while True:
            Cerberus_reg_records.append(self.collect_current())
            if time() > timeout:
                self.inferior_motor.set_angle(0)
                break

        Cerberus_reg_trajectory_records_transform = self.injector_head_transform(Cerberus_reg_records)

        return Cerberus_reg_records, Cerberus_reg_anterior_records_transform, Cerberus_reg_posterior_records_transform, anterior_position, posterior_position

    def anterior_base(self):
        """
        Used to determine the position of the anterior base after positioning on the heart and moving the
        injector head adjacent to it
        """

        recorded = 0
        records = []

        # Define offset distances determined from CAD model (if I do angle align). We are in front of transmitter's
        # front hemisphere
        x0 = 6.0
        y0 = -1.87902209
        z0 = 4.05715298

        # Collect points to warm up EM sensor
        timeout = time() + 3
        while True:
            tmp = self.collect_current()
            if time() > timeout:
                break

        while recorded < 101:  # collect 100 hundred points
            records.append(self.collect_current())
            sleep(self.delay / 1000)  # convert to seconds
            recorded += 1

        anterior_points = np.empty([len(records), 6])  # initialize numpy array for data

        for i in range(len(records)):
            anterior_points[i, 0] = records[i][0]  # x
            anterior_points[i, 1] = records[i][1]  # y
            anterior_points[i, 2] = records[i][2]  # z

            anterior_points[i, 3] = records[i][3]  # r
            anterior_points[i, 4] = records[i][4]  # e
            anterior_points[i, 5] = records[i][5]  # a

        avg_vals = np.mean(anterior_points, axis=0)

        # Convert Euler Angles from degrees to radians
        r = 0.0174533 * avg_vals[3]
        e = 0.0174533 * avg_vals[4]
        a = 0.0174533 * avg_vals[5]

        xsens = avg_vals[0]
        ysens = avg_vals[1]
        zsens = avg_vals[2]
        print("xsens_anterior:", xsens)
        print("ysens_anterior:", ysens)
        print("zsens_anterior:", zsens)

        # Calculate Rotation Matrix from Euler Angles
        m11sens = cos(e) * cos(a)
        m12sens = cos(e) * sin(a)
        m13sens = -sin(e)
        m21sens = -(cos(r) * sin(a)) + (sin(r) * sin(e) * cos(a))
        m22sens = (cos(r) * cos(a)) + (sin(r) * sin(e) * sin(a))
        m23sens = sin(r) * cos(e)
        m31sens = (sin(r) * sin(a)) + (cos(r) * sin(e) * cos(a))
        m32sens = -(sin(r) * cos(a)) + (cos(r) * sin(e) * sin(a))
        m33sens = cos(r) * cos(e)

        rsens = avg_vals[3]
        esens = avg_vals[4]
        asens = avg_vals[5]

        x_ant = xsens + (x0 * m11sens) + (y0 * m21sens) + (z0 * m31sens)
        y_ant = ysens + (x0 * m12sens) + (y0 * m22sens) + (z0 * m32sens)
        z_ant = zsens + (x0 * m13sens) + (y0 * m23sens) + (z0 * m33sens)

        transformed_records = self.injector_head_transform(records)

        return [x_ant, y_ant, z_ant, rsens, esens, asens], records

    def posterior_base(self):
        """
        Used to determine the position of the posterior base after positioning on the heart and
        moving the injector head adjacent to it
        """

        recorded = 0
        records = []

        # Define offset distances determined from CAD model (if I do angle align). We are in front of transmitter's
        # front hemisphere
        x0 = -6.0
        y0 = -1.87902209
        z0 = 4.05715298

        # Collect points to warm up EM sensor
        timeout = time() + 3
        while True:
            tmp = self.collect_current()
            if time() > timeout:
                break

        while recorded < 101:  # collect 100 hundred points
            records.append(self.collect_current())
            sleep(self.delay / 1000)  # convert to seconds
            recorded += 1

        posterior_points = np.empty([len(records), 6])  # initialize numpy array for data

        for i in range(len(records)):
            posterior_points[i, 0] = records[i][0]  # x
            posterior_points[i, 1] = records[i][1]  # y
            posterior_points[i, 2] = records[i][2]  # z

            posterior_points[i, 3] = records[i][3]  # r
            posterior_points[i, 4] = records[i][4]  # e
            posterior_points[i, 5] = records[i][5]  # a

        avg_vals = np.mean(posterior_points, axis=0)

        # Convert Euler Angles from degrees to radians
        r = 0.0174533 * avg_vals[3]
        e = 0.0174533 * avg_vals[4]
        a = 0.0174533 * avg_vals[5]

        xsens = avg_vals[0]
        ysens = avg_vals[1]
        zsens = avg_vals[2]
        print("xsens_posterior:", xsens)
        print("ysens_posterior:", ysens)
        print("zsens_posterior:", zsens)

        # Calculate Rotation Matrix from Euler Angles
        m11sens = cos(e) * cos(a)
        m12sens = cos(e) * sin(a)
        m13sens = -sin(e)
        m21sens = -(cos(r) * sin(a)) + (sin(r) * sin(e) * cos(a))
        m22sens = (cos(r) * cos(a)) + (sin(r) * sin(e) * sin(a))
        m23sens = sin(r) * cos(e)
        m31sens = (sin(r) * sin(a)) + (cos(r) * sin(e) * cos(a))
        m32sens = -(sin(r) * cos(a)) + (cos(r) * sin(e) * sin(a))
        m33sens = cos(r) * cos(e)

        rsens = avg_vals[3]
        esens = avg_vals[4]
        asens = avg_vals[5]

        x_post = xsens + (x0 * m11sens) + (y0 * m21sens) + (z0 * m31sens)
        y_post = ysens + (x0 * m12sens) + (y0 * m22sens) + (z0 * m32sens)
        z_post = zsens + (x0 * m13sens) + (y0 * m23sens) + (z0 * m33sens)

        transformed_records = self.injector_head_transform(records)

        return [x_post, y_post, z_post, rsens, esens, asens], records

    def injector_head_transform(self, reg_records):
        """
        Used to transform the registration points from the sensor location in the
        injector head to the heart's surface
        """
        reg_records_transform = []
        # Define offset distances determined from CAD model (if I do angle align). We are in front of transmitter's
        # front hemisphere
        x0 = 0
        y0 = 0
        z0 = 4.11660638

        for record in reg_records:
            x = record[0]
            y = record[1]
            z = record[2]

            # Convert Euler Angles from degrees to radians
            r = 0.0174533 * record[3]
            e = 0.0174533 * record[4]
            a = 0.0174533 * record[5]

            # Calculate Rotation Matrix from Euler Angles from CAD model offsets
            m11 = cos(e) * cos(a)
            m12 = cos(e) * sin(a)
            m13 = -sin(e)
            m21 = -(cos(r) * sin(a)) + (sin(r) * sin(e) * cos(a))
            m22 = (cos(r) * cos(a)) + (sin(r) * sin(e) * sin(a))
            m23 = sin(r) * cos(e)
            m31 = (sin(r) * sin(a)) + (cos(r) * sin(e) * cos(a))
            m32 = -(sin(r) * cos(a)) + (cos(r) * sin(e) * sin(a))
            m33 = cos(r) * cos(e)

            x_transform = x + (x0 * m11) + (y0 * m21) + (z0 * m31)
            y_transform = y + (x0 * m12) + (y0 * m22) + (z0 * m32)
            z_transform = z + (x0 * m13) + (y0 * m23) + (z0 * m33)

            reg_records_transform.append([x_transform, y_transform, z_transform, record[3], record[4], record[5]])

        return reg_records_transform
