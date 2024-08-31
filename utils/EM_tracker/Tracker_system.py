'''
Author: Aman Ladak
Date: 28-Mar-2022
'''

import EM_tracker.atc3dg_functions as api
from EM_tracker.atc3dg_types import *
import ctypes
import time
from time import time,sleep
import copy

class Tracker_system:
    def __init__(self):
        self.record = DOUBLE_POSITION_ANGLES()
        self.recordPtr = ctypes.pointer(self.record)
        self.system_configuration = None
        self.attached_sensors = None
        self.init_time = None
        self._is_init = False
        self.sensorID = None
        self.delay = None

    def __del__(self):
        self.turn_off_tracker(ignore_error=True)

    def initialize_tracker(self):
        if self.is_init:
            return
        # print("Initializing ATC3DG Tracker System ...")
        error_code = api.InitializeBIRDSystem()
        if error_code != 0:
            self._error_handler(error_code)

        self.sensorID = ctypes.c_ushort(0)
        api.SetSystemParameter(api.SystemParameterType.SELECT_TRANSMITTER,
                               ctypes.pointer(self.sensorID), 2)

        # Measurement properties
        measurement_rate = 80  # only values between 20.0 and 110.0 are legal. 80 Hz is the default and most accurate per the manual
        metric = True  # True for mm, False for inch
        max_range = 36  # inches
        # read in System configuration
        self.system_configurations(measurement_rate, metric, max_range, power_line=60, report_rate=1)


        # # Align angles of sensor
        # AnglesAlignRecord = DOUBLE_ANGLES()
        # pAnglesAlignRecord = ctypes.pointer(AnglesAlignRecord)
        # error_code = api.GetSensorParameter(self.sensorID, api.SensorParameterType.ANGLE_ALIGN, pAnglesAlignRecord, ctypes.sizeof(AnglesAlignRecord))
        # if error_code != 0:
        #     self._error_handler(error_code)
        # print("BEFORE ANGLE ALIGN")
        # print("roll:", AnglesAlignRecord.r)
        # print("elevation:", AnglesAlignRecord.e)
        # print("azimuth:", AnglesAlignRecord.a)

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
        """read and set system and sensor configuration"""

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
        self.delay = delay

    def reset_timer(self):
        self.init_time = time()

    def align_angles(self):
        '''
        Used to align the angles of the EM sensor in the injector head. In our design, we have the EM sensor
        slanted within the injector head (deviation in y-angle). This will also take care of angle deviation in
        x and z. Please perform this step with the injector head placed directly in front of the front hemisphere
        of the transmitter, and have the injector head placed flat on a table.
        '''

        CerbRecord = DOUBLE_POSITION_ANGLES()

        CerbRecord = self.collect_current()

        xangle_offset = CerbRecord[3]
        yangle_offset = CerbRecord[4]
        zangle_offset = CerbRecord[5]

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
        """Returns True if trakstar is initialized"""
        return self._is_init

    def _error_handler(self, error_code):
        print("** Error: ", error_code)
        txt = " " * 500
        pbuffer = ctypes.c_char_p(txt)
        api.GetErrorText(error_code, pbuffer, ctypes.c_int(500),
                         api.MessageType.VERBOSE_MESSAGE)
        print(pbuffer.value)
        self.turn_off_tracker(ignore_error=True)
        raise RuntimeError("** trakSTAR Error")

    def turn_off_tracker(self, ignore_error=True):
        if not self.is_init:
            return
        # print("Turning off Trakstar")
        self.attached_sensors = None
        self.system_configuration = None
        error_code = api.CloseBIRDSystem()
        if error_code != 0 and not ignore_error:
            self._error_handler(error_code)
        self._is_init = False


    def collect_current(self):
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
        recorded = 0
        records = []
        try:
            while True:
                sleep(self.delay/1000)  # convert to seconds
                records.append(self.collect_current())
                print("x:", records[recorded][0], " y:", records[recorded][1], " z:", records[recorded][2], " roll:", records[recorded][3], " elevation:", records[recorded][4], " azimuth:", records[recorded][5])
                recorded += 1
        except KeyboardInterrupt: # Use Ctrl-c to end loop
            print("Tracked turned off")
            pass

        self.turn_off_tracker()
        return records
