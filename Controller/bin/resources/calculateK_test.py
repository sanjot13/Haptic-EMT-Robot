# this file calculates the k values depending on the force readings taken
import numpy as np
import redis
import math as math
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.signal import find_peaks, peak_prominences
import pandas as pd


def sortData(data_array,index1,index2):
    datax = np.array([data_array[index1:index2,1]])
    datay = np.array([data_array[index1:index2,0]])
    
    return datax, datay

def extractData(data):
    # array to store header indexes
    header_index = np.array([],dtype=np.uint32)

    # array to store values
    df = pd.DataFrame()
    
    for i in range(np.shape(data)[0]):
        if math.isnan(data[i,0]) == True:
            header_index = np.append(header_index, i)

    for i in range(header_index.size):
        if i != header_index.size-1:
            displacements, forces = sortData(data, header_index[i]+1, header_index[i+1])
            df2 = pd.DataFrame(np.hstack((displacements.T, forces.T)),columns=list(("Displacement{}".format(i+1), "Forces{}".format(i+1))))
            df = pd.concat((df, df2), axis=1, join='outer')
        else:
            displacements, forces = sortData(data, header_index[i]+1, np.shape(data)[0])
            df2 = pd.DataFrame(np.hstack((displacements.T, forces.T)),columns=list(("Displacement{}".format(i+1), "Forces{}".format(i+1))))
            df = pd.concat((df, df2), axis=1, join='outer')

    return df
            

# defining exponential function to fit to 
def exponential(x, a, b, c):
    return a*np.exp(b*x) - c

# defining linear funtion to fit to
def linear(x, a):
    return a*x

# Settings
force_threshold = -0.1 # threshold for cutting off forces

data = np.genfromtxt("../many_readings.csv", delimiter=',')

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

df = extractData(data)

for i in range(int(len(df.columns)/2)):
    displacement_name = 'Displacement{}'.format(i+1)
    force_name = 'Forces{}'.format(i+1)
    displacement_values = df[displacement_name][df[displacement_name].notnull()].to_numpy()
    force_values = df[force_name][df[force_name].notnull()].to_numpy()

    # used to get forces agaisnt displacement graphs
    # displacement_values = displacement_values[force_values < force_threshold]
    # force_values = force_values[force_values < force_threshold]
    # force_values = -force_values
    # force_values = force_values - force_values[0]
    # displacement_values = -(displacement_values - np.max(displacement_values))
    # plt.figure()
    # plt.plot(displacement_values, force_values)

    # plot forces only
    time = np.linspace(0,force_values.size/1000, force_values.size)
    print(force_values.size)
    peaks, _ = find_peaks(-force_values, prominence=1)
    # print(max(-force_values))
    print(force_values[peaks])
    plt.plot(time,-force_values)
    plt.ylim((-0.5,6))


plt.show()