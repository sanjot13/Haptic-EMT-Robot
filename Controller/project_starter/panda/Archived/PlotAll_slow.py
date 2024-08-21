import redis
import numpy as np
import ast
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def redisCatchTorqueAndTime(r, value_key_name, time_key_name='your_time_key_name'):

    # Get all value keys
    value1_keys = r.keys('*')
    value2_keys = r.keys('*')
    value3_keys = r.keys('*')
    value4_keys = r.keys('*')
    value5_keys = r.keys('*')
    time_keys = r.keys('*')

    # Filter value keys by name
    filtered_value1_keys = [key for key in value1_keys if key.decode('latin-1') == value_key_name[0]]
    filtered_value2_keys = [key for key in value2_keys if key.decode('latin-1') == value_key_name[1]]
    filtered_value3_keys = [key for key in value3_keys if key.decode('latin-1') == value_key_name[2]]
    filtered_value4_keys = [key for key in value4_keys if key.decode('latin-1') == value_key_name[3]]
    filtered_value5_keys = [key for key in value5_keys if key.decode('latin-1') == value_key_name[4]]
    filtered_time_keys = [key for key in time_keys if key.decode('latin-1') == time_key_name]

    # Use a pipeline to retrieve values for selected keys
    with r.pipeline() as pipe:
        for value1_key, value2_key, value3_key, value4_key, value5_key, time_key in zip(filtered_value1_keys, filtered_value2_keys, filtered_value3_keys, 
                                                                            filtered_value4_keys, filtered_value5_keys, filtered_time_keys):
            pipe.get(value1_key)
            pipe.get(value2_key)
            pipe.get(value3_key)
            pipe.get(value4_key)
            pipe.get(value5_key)
            pipe.get(time_key)
        AllValues = pipe.execute()
    
    # Convert string representation of list to actual list of floats
    values1 = [ast.literal_eval(value1.decode('utf-8')) if value1 else None for value1 in AllValues[::6]]
    values2 = [ast.literal_eval(value2.decode('utf-8')) if value2 else None for value2 in AllValues[1::6]]
    values3 = [ast.literal_eval(value3.decode('utf-8')) if value3 else None for value3 in AllValues[2::6]]
    values4 = [ast.literal_eval(value4.decode('utf-8')) if value4 else None for value4 in AllValues[3::6]]
    values5 = [ast.literal_eval(value4.decode('utf-8')) if value4 else None for value4 in AllValues[4::6]]
    times = [ast.literal_eval(time.decode('utf-8')) if time else None for time in AllValues[5::6]]
    
    # Flatten the lists if necessary
    flat_values1 = [item for sublist in values1 for item in sublist] if values1 else None
    flat_values2 = [item for sublist in values2 for item in sublist] if values2 else None
    flat_values3 = [item for sublist in values3 for item in sublist] if values3 else None
    flat_values4 = [item for sublist in values4 for item in sublist] if values4 else None
    flat_values5 = [item for sublist in values5 for item in sublist] if values5 else None

    # Convert the lists of values and times to NumPy arrays
    values1_array = np.array(flat_values1)
    values2_array = np.array(flat_values2)
    values3_array = np.array(flat_values3)
    values4_array = np.array(flat_values4)
    values5_array = np.array(flat_values5)
    times_array = np.array(times)

    return values1_array, values2_array, values3_array, values4_array, values5_array, times_array

def animate(i,Time, Torque1, Torque2, Torque3, Torque4, Torque5, Torque6, Torque7, XForce, YForce, ZForce, 
            XPosition, YPosition, ZPosition, XForceEe, YForceEe, ZForceEe, XMomentsEe, YMomentsEe, ZMomentsEe, r):
    TorqueValue, WorldForceValue, PositionValue, EeForceValue, EeMomentValue, TimeValue = redisCatchTorqueAndTime(r, 
                                                                                                   value_key_name=['cs225a::robot_command_torques::PANDA','world_Sensed_Force','cs225a::simviz::position', 'ee_Sensed_Force', 'ee_Sensed_Moment'], 
                                                                                                   time_key_name='Time')
    Time.append(TimeValue)

    Torque1.append(TorqueValue[0])
    Torque2.append(TorqueValue[1])
    Torque3.append(TorqueValue[2])
    Torque4.append(TorqueValue[3])
    Torque5.append(TorqueValue[4])
    Torque6.append(TorqueValue[5])
    Torque7.append(TorqueValue[6])

    XForce.append(WorldForceValue[0])
    YForce.append(WorldForceValue[1])
    ZForce.append(WorldForceValue[2])
    
    XPosition.append(PositionValue[0])
    YPosition.append(PositionValue[1])
    ZPosition.append(PositionValue[2])

    XForceEe.append(EeForceValue[0])
    YForceEe.append(EeForceValue[1])
    ZForceEe.append(EeForceValue[2])

    XMomentsEe.append(EeMomentValue[0])
    YMomentsEe.append(EeMomentValue[1])
    ZMomentsEe.append(EeMomentValue[2])
    
    axs[0,0].plot(Time,Torque1,color='b')
    axs[0,0].plot(Time,Torque2,color='g')
    axs[0,0].plot(Time,Torque3,color='r')
    axs[0,0].plot(Time,Torque4,color='c')
    axs[0,0].plot(Time,Torque5,color='m')
    axs[0,0].plot(Time,Torque6,color='y')
    axs[0,0].plot(Time,Torque7,color='k')

    axs[0,1].plot(Time,XForce,color="b")
    axs[0,1].plot(Time,YForce,color="g")
    axs[0,1].plot(Time,ZForce,color="r")

    axs[1,0].plot(Time,XPosition,color="b")
    axs[1,0].plot(Time,YPosition,color="g")
    axs[1,0].plot(Time,ZPosition,color="r")

    axs[1,1].plot(Time,XForceEe,color="b")
    axs[1,1].plot(Time,YForceEe,color="g")
    axs[1,1].plot(Time,ZForceEe,color="r")

    axs[2,0].plot(Time,XMomentsEe,color="b")
    axs[2,0].plot(Time,YMomentsEe,color="g")
    axs[2,0].plot(Time,ZMomentsEe,color="r")

    axs[0,0].set_xlabel('Time [s]')
    axs[0,0].set_ylabel('Torques [Nm]')
    axs[0,0].legend(['Joint 1 Torques','Joint 2 Torques','Joint 3 Torques','Joint 4 Torques','Joint 5 Torques','Joint 6 Torques','Joint 7 Torques'])
    axs[0,0].set_title('Commanded Torques')

    axs[0,1].set_xlabel('Time [s]')
    axs[0,1].set_ylabel('Force [N]')
    axs[0,1].legend(['X Force', 'Y Force', 'Z Force'])
    axs[0,1].set_title('World Sensed Forces')

    axs[1,0].set_xlabel('Time [s]')
    axs[1,0].set_ylabel('Position [m]')
    axs[1,0].legend(['X Position', 'Y Position', 'Z Position'])
    axs[1,0].set_title('End-effector position')

    axs[1,1].set_xlabel('Time [s]')
    axs[1,1].set_ylabel('Force [N]')
    axs[1,1].legend(['X Force', 'Y Force', 'Z Force'])
    axs[1,1].set_title('Ee Sensed Forces')

    axs[2,0].set_xlabel('Time [s]')
    axs[2,0].set_ylabel('Force [N]')
    axs[2,0].legend(['X Moment', 'Y Moment', 'Z Moment'])
    axs[2,0].set_title('Ee Sensed Moments')

if __name__ == "__main__":
    fig, axs = plt.subplots(3,2, figsize=(17,12))

    time = []
    Torque1 = []
    Torque2 = []
    Torque3 = []
    Torque4 = []
    Torque5 = []
    Torque6 = []
    Torque7 = []
    XForce = []
    YForce = []
    ZForce = []
    XPosition = []
    YPosition = []
    ZPosition = []
    XForceEe = []
    YForceEe = []
    ZForceEe = []
    XMomentsEe = []
    YMomentsEe = []
    ZMomentsEe = []


    r = redis.Redis(host='localhost', port=6379, db=0)

    r.set("Ready_Signal", "False")
    
    while ast.literal_eval(r.get("Ready_Signal").decode('utf-8')) == False:
        if ast.literal_eval(r.get("Ready_Signal").decode('utf-8')) == True:
            break
    
    ani = animation.FuncAnimation(fig,animate,fargs=(time, Torque1, Torque2, Torque3, Torque4, Torque5, Torque6, Torque7, XForce, YForce, ZForce, 
                                                     XPosition, YPosition, ZPosition, XForceEe, YForceEe, ZForceEe, XMomentsEe, YMomentsEe, ZMomentsEe, r),interval=1)

    plt.show()