# this file calculates the k values depending on the force readings taken
import numpy as np
import redis

data_file = open("../readings.txt", "r")

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

# intialize empty arrays for readings
force_values = []
displacement_values = []

for line in data_file.readlines():
    line = line.strip()
    if line[0:4] == "F1: " or line[0:4] ==  "F2: ":
        line = line[4:]
        force_values.append(line)
    if line[0:4] == "D1: " or line[0:4] ==  "D2: ":
        line = line[4:]
        displacement_values.append(line)

# convert arrays to numpy arrays
force_values = np.asarray(force_values, dtype=float)
displacement_values = np.asarray(displacement_values, dtype=float)

for i in range(3):
    if i == 0:
        k1 = (force_values[i]-force_values[i+1])/(displacement_values[i]-displacement_values[i+1])
        r.set("k1", "{}".format(k1))
    elif i == 1:
        k2 = (force_values[i+1]-force_values[i+2])/(displacement_values[i+1]-displacement_values[i+2])
        r.set("k2", "{}".format(k2))
    elif i == 2:
        k3 = (force_values[i+2]-force_values[i+3])/(displacement_values[i+2]-displacement_values[i+3])
        r.set("k3", "{}".format(k3))

data_file.close()