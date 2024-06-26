#!/usr/bin/env python

"""This example demonstrates how to use the inverse3 position control. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import time
connected_devices = HaplyHardwareAPI.detect_inverse3s()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
print("connected to device {}".format(response_to_wakeup["device_id"]))
start_time = time.perf_counter()
loop_time = 0.001  # 1ms
center_workspace = [0.0186, -0.153, 0.151]
target_position = center_workspace  # 1N in the y direction
while True:
    position, velocity = inverse3.end_effector_position(target_position)
    print("position: {}".format(position))
    while time.perf_counter() - start_time < loop_time:  # wait for loop time to be reached
        pass
    start_time = time.perf_counter()
