import numpy as np
import matplotlib.pyplot as plt
# NOTE:  Sync frequency of Kinova and Bota data

KINOVA_SAMPLING_RATE = 40
kinova_delay = 7.6 # s
# Alternative Loading method with python instead of numpy
# f = open("test.txt", 'r')

# Load Kinova & Bota data
# kinova_data = np.loadtxt("/home/sunny/Documents/Kinova/scripts/cmake-build-release/kinova_force_wrenches.txt", delimiter=",")
kinova_data = np.loadtxt("/home/sunny/Documents/Kinova/scripts/cmake-build-release/kinova_forces.txt", delimiter=",")
bota_data = np.load("/home/sunny/Documents/Haptic Reference Sensor/data_teensy.npz")
# bota_data = np.load('data_bota.npz')

# Splice data into variables: time & force
kinova_times = kinova_data[:,0]
kinova_times = kinova_data[:,0] / KINOVA_SAMPLING_RATE + kinova_delay
kinova_forces = kinova_data[:,1]
bota_times = bota_data['x']
bota_forces = bota_data['y']

# Plot values
plt.plot(bota_times, bota_forces, 'r', lw=2, label = "Teensy Sensor")
plt.plot(kinova_times, kinova_forces, 'b', lw=2, label = "Kinova Estimates")
plt.xlabel("Time (s)")
plt.ylabel(("Force (N)"))
# plt.title(("Kinova Wrench Estimates vs Bota Sensor"))
# plt.title(("Kinova Joint Force Estimates"))
plt.title(("Kinova Joint Force Estimates vs Teensy Sensor"))
plt.legend()
plt.show()