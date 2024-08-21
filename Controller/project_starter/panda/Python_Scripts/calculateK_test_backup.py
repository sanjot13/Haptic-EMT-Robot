# this file calculates the k values depending on the force readings taken
import numpy as np
import redis
import math as math
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def extractData(data_array,index1,index2):
    datax = data_array[index1:index2,1]
    datay = data_array[index1:index2,0]
    
    return datax, datay

# defining exponential function to fit to 
def exponential(x, a, b, c):
    return a*np.exp(b*x) - c

# defining linear funtion to fit to
def linear(x, a):
    return a*x

# Settings
force_threshold = -0.1 # threshold for cutting off forces

data = np.genfromtxt("../many_readings.csv", delimiter=',')

# array to store header indexes
header_index = np.array([],dtype=np.uint32)

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

for i in range(np.shape(data)[0]):
    if math.isnan(data[i,0]) == True:
        header_index = np.append(header_index, i)

displacements, forces = extractData(data, header_index[0]+1, header_index[1])
displacements2, forces2 = extractData(data, header_index[1]+1, header_index[2])
displacements3, forces3 = extractData(data, header_index[2]+1, np.shape(data)[0])

# filter out forces which are below a certain threshold
displacements = displacements[forces < force_threshold]
forces = forces[forces < force_threshold]
displacements2 = displacements2[forces2 < force_threshold]
forces2 = forces2[forces2 < force_threshold]
displacements3 = displacements3[forces3 < force_threshold]
forces3 = forces3[forces3 < force_threshold]

# flip the sign on forces and zero them out by first reading
forces = -forces
forces = forces - forces[0]
forces2 = -forces2
forces2 = forces2 - forces2[0]
forces3 = -forces3
forces3 = forces3 - forces3[0]

# zero out displacements
displacements = -(displacements - np.max(displacements))
displacements2 = -(displacements2 - np.max(displacements2))
displacements3 = -(displacements3 - np.max(displacements3))

# print(np.shape(displacements))
# print(np.shape(displacements2))
# print(np.shape(displacements3))

# first figure
plt.scatter(displacements, forces)
param_lin, param_cov_lin = curve_fit(linear, displacements, forces)
sigma = np.ones(np.size(forces))
sigma[0] = 0.001
param_exp, param_cov_exp = curve_fit(exponential, displacements, forces, sigma=sigma)
plt.plot(displacements, param_exp[0]*np.exp(param_exp[1]*displacements) - param_exp[2], label='expn fit')
plt.plot(displacements, param_lin[0]*displacements, label='linear fit')
plt.legend()

print(param_exp)

ss_res = np.sum((forces - (param_exp[0]*np.exp(param_exp[1]*displacements) - param_exp[2])) ** 2)
ss_tot = np.sum((forces - np.mean(forces)) ** 2)
r2_expn = 1 - (ss_res / ss_tot)
ss_res = np.sum((forces - (param_lin[0]*displacements)) ** 2)
ss_tot = np.sum((forces - np.mean(forces)) ** 2)
r2_lin = 1 - (ss_res / ss_tot)
print(r2_expn)
print(r2_lin)

# second figure
plt.figure()
plt.scatter(displacements2,forces2)
sigma = np.ones(np.size(forces2))
sigma[0] = 0.001
param_exp, param_cov_exp = curve_fit(exponential, displacements2, forces2, sigma=sigma)
param_lin, param_cov_lin = curve_fit(linear, displacements2, forces2)
plt.plot(displacements2, param_exp[0]*np.exp(param_exp[1]*displacements2) - param_exp[2], label='expn fit')
plt.plot(displacements2, param_lin[0]*displacements2, label='linear fit')
plt.legend()

print(param_exp)

ss_res = np.sum((forces2 - (param_exp[0]*np.exp(param_exp[1]*displacements2) - param_exp[2])) ** 2)
ss_tot = np.sum((forces2 - np.mean(forces2)) ** 2)
r2_expn = 1 - (ss_res / ss_tot)
ss_res = np.sum((forces2 - (param_lin[0]*displacements2)) ** 2)
ss_tot = np.sum((forces2 - np.mean(forces2)) ** 2)
r2_lin = 1 - (ss_res / ss_tot)
print(r2_expn)
print(r2_lin)

# third figure
plt.figure()
plt.scatter(displacements3,forces3)
# param_exp, param_cov_exp = curve_fit(exponential, displacements3, forces3)
param_lin, param_cov_lin = curve_fit(linear, displacements3, forces3)
# plt.plot(displacements3, param_exp[0]*np.exp(param_exp[1]*displacements3) - param_exp[2], label='expn fit')
plt.plot(displacements3, param_lin[0]*displacements3, label='linear fit')
plt.legend()

print(param_lin)
plt.show()