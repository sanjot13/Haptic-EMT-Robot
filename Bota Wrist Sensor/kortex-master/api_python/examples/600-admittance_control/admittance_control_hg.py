#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import time
import sys
import os
import threading
import math
import struct

from collections import namedtuple

# serial device
import serial
from crc import CrcCalculator, Configuration

# KORTEX
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20
KP_TR = 0.003
KP_ROT = 15
FZ_DES = -10


class SerialExample:

    BOTA_PRODUCT_CODE = 123456
    BAUDERATE = 460800
    SINC_LENGTH = 2048
    CHOP_ENABLE = 0
    FAST_ENABLE = 0
    FIR_DISABLE = 1
    TEMP_COMPENSATION = 0 # 0: Disabled (recommended), 1: Enabled
    USE_CALIBRATION = 1 # 1: calibration matrix active, 0: raw measurements
    DATA_FORMAT = 0 # 0: binary, 1: CSV
    BAUDERATE_CONFIG = 4 # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    FRAME_HEADER = b'\xAA'
    # Note that the time step is set according to the sinc filter size!
    time_step = 0.01;

    def __init__(self, port, base, base_cyclic):
        self._port = port
        self._ser = serial.Serial()
        self._pd_thread_stop_event = threading.Event()
        self._kinova_base = base
        self._kinova_base_cyclic = base_cyclic
        DeviceSet = namedtuple('DeviceSet', 'name product_code config_func')
        self._expected_device_layout = {0: DeviceSet('BFT-SENS-SER-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}

    def bota_sensor_setup(self):
        print("bota_sensor_setup")
        # Wait for streaming of data
        self._ser.read_until(bytes('App Init', 'ascii'))
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Go to CONFIG mode
        cmd = bytes('C', 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,C,0', 'ascii'))

        # Communication setup
        comm_setup = f"c,{self.TEMP_COMPENSATION},{self.USE_CALIBRATION},{self.DATA_FORMAT},{self.BAUDERATE_CONFIG}"
        #print(comm_setup)
        cmd = bytes(comm_setup, 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,c,0', 'ascii'))
        time_step = 0.00001953125*self.SINC_LENGTH
        print("Timestep: {}".format(time_step))

        # Filter setup
        filter_setup = f"f,{self.SINC_LENGTH},{self.CHOP_ENABLE},{self.FAST_ENABLE},{self.FIR_DISABLE}"
        #print(filter_setup)
        cmd = bytes(filter_setup, 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,f,0', 'ascii'))

        # Go to RUN mode
        cmd = bytes('R', 'ascii')
        self._ser.write(cmd)
        self._ser.read_until(bytes('r,0,R,0', 'ascii'))

    def _processdata_thread(self):
        while not self._pd_thread_stop_event.is_set():
            frame_synced = False
            crc16X25Configuration = Configuration(16, 0x1021, 0xFFFF, 0xFFFF, True, True)
            crc_calculator = CrcCalculator(crc16X25Configuration)

            command = Base_pb2.TwistCommand()

            command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
            command.duration = 0

            while not frame_synced and not self._pd_thread_stop_event.is_set():
                possible_header = self._ser.read(1)
                if self.FRAME_HEADER == possible_header:
                    #print(possible_header)
                    data_frame = self._ser.read(34)
                    crc16_ccitt_frame = self._ser.read(2)

                    #status = struct.unpack_from('H', data_frame, 0)[0]
                    #print("Status {}".format(status))

                    Fx_offset = -struct.unpack_from('f', data_frame, 2)[0]
                    Fy_offset = -struct.unpack_from('f', data_frame, 6)[0]
                    Fz_offset = struct.unpack_from('f', data_frame, 10)[0]
                    Mx_offset = -struct.unpack_from('f', data_frame, 14)[0]
                    My_offset = -struct.unpack_from('f', data_frame, 18)[0]
                    Mz_offset = struct.unpack_from('f', data_frame, 22)[0]
                    print("Fx {}".format(Fx_offset))
                    print("Fy {}".format(Fy_offset))
                    print("Fz {}".format(Fz_offset))
                    print("Mx {}".format(Mx_offset))
                    print("My {}".format(My_offset))
                    print("Mz {}".format(Mz_offset))

                    #timestamp = struct.unpack_from('I', data_frame, 26)[0]
                    #print("Timestamp {}".format(timestamp))

                    #temperature = struct.unpack_from('f', data_frame, 30)[0]
                    #print("Temperature {}\n".format(temperature))

                    crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                    # print("crc16_ccitt {}".format(crc16_ccitt))
                    checksum = crc_calculator.calculate_checksum(data_frame)
                    # print("checksum {}".format(checksum))
                    if checksum == crc16_ccitt:
                        print("Frame synced")
                        frame_synced = True;
                    else:
                        self._ser.read(1)

            t_start = time.time()
            t_start_track = time.time()
            isTrackingStarted = False;
            t_now = time.time()
            while frame_synced and not self._pd_thread_stop_event.is_set():
                start_time = time.perf_counter()
                frame_header = self._ser.read(1)

                if frame_header != self.FRAME_HEADER:
                    print("Lost sync")
                    frame_synced = False
                    break

                data_frame = self._ser.read(34)
                crc16_ccitt_frame = self._ser.read(2)

                crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                checksum = crc_calculator.calculate_checksum(data_frame)
                if checksum != crc16_ccitt:
                    print("CRC mismatch received")
                    break

                status = struct.unpack_from('H', data_frame, 0)[0]
                # print("Status {}".format(status))
                #
                Fx = -struct.unpack_from('f', data_frame, 2)[0]
                # print("Fx {}".format(Fx))
                Fy = -struct.unpack_from('f', data_frame, 6)[0]
                # print("Fy {}".format(Fy))
                Fz = struct.unpack_from('f', data_frame, 10)[0]
                # print("Fz {}".format(Fz))
                Mx = -struct.unpack_from('f', data_frame, 14)[0]
                # print("Mx {}".format(Mx))
                My = -struct.unpack_from('f', data_frame, 18)[0]
                # print("My {}".format(My))
                Mz = struct.unpack_from('f', data_frame, 22)[0]
                # print("Mz {}".format(Mz))

                timestamp = struct.unpack_from('I', data_frame, 26)[0]
                # print("Timestamp {}, {}".format(timestamp, t_now-t_start))

                temperature = struct.unpack_from('f', data_frame, 30)[0]
                # print("Temperature {}\n".format(temperature))

                # feedback = self._kinova_base_cyclic.RefreshFeedback()

                t_now = time.time()
                twist = command.twist
                if isTrackingStarted:
                    t_start_track = t_now - t_start;
                    twist.linear_x = KP_TR * (Fx - Fx_offset)
                    if t_start_track % 60 < 30:
                        twist.linear_y = KP_TR * (Fy - Fy_offset) - 0.005
                    else:
                        twist.linear_y = KP_TR * (Fy - Fy_offset) + 0.005
                else:
                    twist.linear_x = KP_TR * (Fx - Fx_offset)
                    twist.linear_y = KP_TR * (Fy - Fy_offset)
                twist.linear_z = KP_TR * ((Fz - Fz_offset))
                # print("F tracking {}".format((Fz - Fz_offset) - FZ_DES))
                # if ((Fz - Fz_offset) - FZ_DES) < -FZ_DES*0.2 and not isTrackingStarted:
                #     isTrackingStarted = True
                # print("Z pose {}".format(feedback.base.tool_pose_z))
                twist.angular_x = KP_ROT * (Mx - Mx_offset)
                twist.angular_y = KP_ROT * (My - My_offset)
                twist.angular_z = KP_ROT * (Mz - Mz_offset)

                self._kinova_base.SendTwistCommand(command)

                time_diff = time.perf_counter() - start_time
                if time_diff < self.time_step:
                    SerialExample._sleep((self.time_step - time_diff)/2)


    def _my_loop(self):

        try:
            while 1:
                print('Run my loop')

                time.sleep(5.0)

        except KeyboardInterrupt:
            # ctrl-C abort handling
            print('stopped')
            self._kinova_base.Stop()
            time.sleep(1)


    def run(self):

        self._ser.baudrate = self.BAUDERATE
        self._ser.port = self._port
        self._ser.timeout = 10

        try:
            self._ser.open()
        except:
            raise SerialExampleError('Could not open port')

        if not self._ser.is_open:
            raise SerialExampleError('Could not open port')

        self.bota_sensor_setup()

        #check_thread = threading.Thread(target=self._check_thread)
        #check_thread.start()
        proc_thread = threading.Thread(target=self._processdata_thread)
        proc_thread.start()


        device_running = True

        if device_running:
            self._my_loop()

        self._pd_thread_stop_event.set()
        proc_thread.join()
        #check_thread.join()

        self._ser.close()

        if not device_running:
            raise SerialExampleError('Device is not running')

    @staticmethod
    def _sleep(duration, get_now=time.perf_counter):
        now = get_now()
        end = now + duration
        while now < end:
            now = get_now()

class SerialExampleError(Exception):
    def __init__(self, message):
        super(SerialExampleError, self).__init__(message)
        self.message = message


# Create closure to set an event after an END or an ABORT
def check_for_sequence_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications on a sequence

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e = e):
        event_id = notification.event_identifier
        task_id = notification.task_index
        if event_id == Base_pb2.SEQUENCE_TASK_COMPLETED:
            print("Sequence task {} completed".format(task_id))
        elif event_id == Base_pb2.SEQUENCE_ABORTED:
            print("Sequence aborted with error {}:{}"\
                .format(\
                    notification.abort_details,\
                    Base_pb2.SubErrorCodes.Name(notification.abort_details)))
            e.set()
        elif event_id == Base_pb2.SEQUENCE_COMPLETED:
            print("Sequence completed.")
            e.set()
    return check

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)

    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def create_cartesian_action(base_cyclic):

    print("Creating Cartesian action")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y   # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z    # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + 90# (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y# (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + 90 # (degrees)

    return action

def example_create_sequence(base, base_cyclic):
    print("Creating Action for Sequence")

    actuator_count = base.GetActuatorCount().count
    cartesian_action = create_cartesian_action(base_cyclic)

    print("Creating Sequence")
    sequence = Base_pb2.Sequence()
    sequence.name = "Example sequence"

    print("Appending Actions to Sequence")
    task_1 = sequence.tasks.add()
    task_1.group_identifier = 0
    task_1.action.CopyFrom(cartesian_action)

    e = threading.Event()
    notification_handle = base.OnNotificationSequenceInfoTopic(
        check_for_sequence_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Creating sequence on device and executing it")
    handle_sequence = base.CreateSequence(sequence)
    base.PlaySequence(handle_sequence)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished


def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True
        success &= example_move_to_home_position(base)
        success &= example_create_sequence(base, base_cyclic)

        SerialExample('/dev/ttyUSB0', base, base_cyclic).run()

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
