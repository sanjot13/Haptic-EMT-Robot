#!/usr/bin/env python

"""This example demonstrates how to read the inverse3 position. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import redis as r
import time
import numpy as np

tick_count = 0

class Handle(HaplyHardwareAPI.Handle):
    def __init__(self, com):
        HaplyHardwareAPI.Handle.__init__(self, com)
        self.Hold_Position = False
        self.tick_count = 0
        self.previous = 0
        self.current = 0

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

        # reads rising edge
        self.current = user_data[0]
        if self.previous == 0 and self.current == 1:
            self.Hold_Position = not self.Hold_Position
            self.target_position = position
        self.previous = user_data[0]

        # if user_data[0] == 1:
        #     self.Button_Pressed = "True"
        # if user_data[0] == 0:
        #     self.tick_count  = 0
        # if user_data[0] == 1:
        #     self.tick_count  += 1
        # if self.tick_count  > 10:
        #     print("Trigger")
        #     self.Hold_Position = not self.Hold_Position
        #     self.tick_count  = 0
        # self.Button_Pressed = "False"
        
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
while True:
    print(handle.Hold_Position)
    # flip axes before sending
    if handle.Hold_Position == False:
        position, velocity = inverse3.end_effector_force()
        print("position: {}".format(position))
    elif handle.Hold_Position == True:
        position, velocity = inverse3.end_effector_position(handle.target_position)
    
    kinova_position = [(-position[1]+0.0329865),(position[0]-0.0325719),position[2]-0.134252]
    r.set("Desired_Position", "{}".format(kinova_position))

    handle.RequestStatus()
    handle.Receive()
