#!/usr/bin/env python

"""This example demonstrates how to read the inverse3 position. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import redis as r
import time
import numpy as np

class Handle(HaplyHardwareAPI.Handle):
    def __init__(self, com):
        HaplyHardwareAPI.Handle.__init__(self, com)

    def OnReceiveHandleInfo(self, data_remaining, device_id, device_model, hardware_version, firmware_version):
        print("Tool info received, device ID: " + str(device_id) + " device model: " + str(device_model) +
              " hardware version: " + str(hardware_version) + " firmware version: " + str(firmware_version))

    def OnReceiveHandleStatusMessage(self, device_id, quaternion, error_flag, hall_effect_sensor_level, user_data_length, user_data):
        print("Tool status received")
        print("Device ID: " + str(device_id))
        print("Error flag: " + str(error_flag))
        print("Hall effect sensor level: " + str(hall_effect_sensor_level))
        print("quaternion: " + str(quaternion[0]) + " " + str(
            quaternion[1]) + " " + str(quaternion[2]) + " " + str(quaternion[3]))
        r.set("Desired_Quaternion", "{}".format(quaternion))
        for i in range(user_data_length):
            print("user_data: " + str(user_data[i]))
        if user_data[0] == 1:
            r.set("Button_Status", "True")
        else:
            r.set("Button_Status", "False")

    def OnReceiveHandleErrorResponse(self):
        print("Tool error received")

    def RequestStatus(self):
        HaplyHardwareAPI.Handle.RequestStatus(self)

connected_devices = HaplyHardwareAPI.detect_inverse3s()
connected_handles = HaplyHardwareAPI.detect_handles()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
# print the response to the wakeup command
for key in response_to_wakeup:
    print(key, response_to_wakeup[key])
handle_stream = HaplyHardwareAPI.SerialStream(connected_handles[0])
handle = Handle(handle_stream)
handle.SendDeviceWakeup()
handle.Receive()

# print the response to the wakeup command
print("connected to device {}".format(response_to_wakeup["device_id"]))


r = r.Redis(host='localhost', port=6379, decode_responses=True)


start_time = time.perf_counter()
loop_time = 0.001  # 1ms

while True:
    # Reading XYZ forces from the Bota sensor
    force = r.get("cs225a::robot_command_forces::PANDA").strip('[]')
    force = np.array(np.fromstring(force, sep=','))

    z = force[2] * 0.5
    print("Z force: {}".format(z))

    # Send Z force to the Haptic Interface
    position, velocity = inverse3.end_effector_force([0,0,z])


    # position, velocity = inverse3.end_effector_force()
    print("position: {}".format(position))
    # flip axes before sending
    position = [(-position[1]+0.0329865),(position[0]-0.0325719),position[2]-0.134252]
    r.set("Desired_Position", "{}".format(position))
    handle.RequestStatus()
    handle.Receive()

    # wait for loop time to be reached
    while time.perf_counter() - start_time < loop_time:  
        pass
    start_time = time.perf_counter()
