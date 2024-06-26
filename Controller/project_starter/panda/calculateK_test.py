# this file calculates the k values depending on the force readings taken
import numpy as np
import redis
import math as math
import matplotlib.pyplot as plt

def extractData(data_array,index1,index2):
    datax = data_array[index1:index2,1]
    datay = data_array[index1:index2,0]
    
    return datax, datay

data = np.genfromtxt("../many_readings.csv", delimiter=',')

# array to store header indexes
header_index = np.array([],dtype=np.uint32)

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

for i in range(np.shape(data)[0]):
    if math.isnan(data[i,0]) == True:
        header_index = np.append(header_index, i)

print(type(header_index[0]))
displacements, forces = extractData(data, header_index[0]+1, header_index[1])
displacements2, forces2 = extractData(data, header_index[1]+1, header_index[2])
displacements3, forces3 = extractData(data, header_index[2]+1, np.shape(data)[0])

plt.scatter(displacements, forces)
plt.figure()
plt.scatter(displacements2,forces2)
plt.figure()
plt.scatter(displacements3,forces3)

print(np.shape(displacements))
print(np.shape(displacements2))
print(np.shape(displacements3))

plt.show()