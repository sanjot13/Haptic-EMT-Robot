import redis
import numpy as np
import ast
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time as t
import matplotlib.style as mplstyle
mplstyle.use('fast')

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
            XPosition, YPosition, ZPosition, XForceEe, YForceEe, ZForceEe, XMomentsEe, YMomentsEe, ZMomentsEe,r):
    # Used to calculate loop time
    # time1 = t.time()

    TorqueValue, WorldForceValue, PositionValue, EeForceValue, EeMomentValue, TimeValue = redisCatchTorqueAndTime(r, 
                                                                                                   value_key_name=['cs225a::robot_command_torques::PANDA','world_Sensed_Force','cs225a::simviz::position', 'ee_Sensed_Force', 'ee_Sensed_Moment'], 
                                                                                                   time_key_name='Time')
    
    Torque1.pop(0)
    Torque1.append(TorqueValue[0])
    Torque2.pop(0)
    Torque2.append(TorqueValue[1])
    Torque3.pop(0)
    Torque3.append(TorqueValue[2])
    Torque4.pop(0)
    Torque4.append(TorqueValue[3])
    Torque5.pop(0)
    Torque5.append(TorqueValue[4])
    Torque6.pop(0)
    Torque6.append(TorqueValue[5])
    Torque7.pop(0)
    Torque7.append(TorqueValue[6])

    XForce.pop(0)
    XForce.append(WorldForceValue[0])
    YForce.pop(0)
    YForce.append(WorldForceValue[1])
    ZForce.pop(0)
    ZForce.append(WorldForceValue[2])
    
    XPosition.pop(0)
    XPosition.append(PositionValue[0])
    YPosition.pop(0)
    YPosition.append(PositionValue[1])
    ZPosition.pop(0)
    ZPosition.append(PositionValue[2])

    XForceEe.pop(0)
    XForceEe.append(EeForceValue[0])
    YForceEe.pop(0)
    YForceEe.append(EeForceValue[1])
    ZForceEe.pop(0)
    ZForceEe.append(EeForceValue[2])

    XMomentsEe.pop(0)
    XMomentsEe.append(EeMomentValue[0])
    YMomentsEe.pop(0)
    YMomentsEe.append(EeMomentValue[1])
    ZMomentsEe.pop(0)
    ZMomentsEe.append(EeMomentValue[2])
    
    line1_1.set_ydata(Torque1)
    line1_1.set_xdata(Time)
    line1_2.set_ydata(Torque2)
    line1_3.set_ydata(Torque3)
    line1_4.set_ydata(Torque4)
    line1_5.set_ydata(Torque5)
    line1_6.set_ydata(Torque6)
    line1_7.set_ydata(Torque7)

    line2_1.set_ydata(XForce)
    line2_2.set_ydata(YForce)
    line2_3.set_ydata(ZForce)

    line3_1.set_ydata(XPosition)
    line3_2.set_ydata(YPosition)
    line3_3.set_ydata(ZPosition)

    line4_1.set_ydata(XForceEe)
    line4_2.set_ydata(YForceEe)
    line4_3.set_ydata(ZForceEe)

    line5_1.set_ydata(XMomentsEe)
    line5_2.set_ydata(YMomentsEe)
    line5_3.set_ydata(ZMomentsEe)

    # Used to calculate loop time
    # time2 = t.time()
    # print(time2-time1)

    # print(Time)

    return line1_1, line1_2, line1_3, line1_4, line1_5, line1_6, line1_7, line2_1, line2_2, line2_3, line3_1, line3_2, line3_3, line4_1, line4_2, line4_3, line5_1, line5_1, line5_3

if __name__ == "__main__":
    fig, axs = plt.subplots(3,2)
    plt.tight_layout()
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    x_len = 400
    axs[0,0].set_ylim([-40,40]) # set ranges for joint torques
    axs[0,1].set_ylim([-10,10]) # set ranges for forces in world frame
    axs[1,0].set_ylim([-10,10])
    axs[1,1].set_ylim([-10,10])
    axs[2,0].set_ylim([-10,10])
    axs[2,1].set_ylim([-10,10])

    xs = list(range(0, x_len))
    ys = [0] * x_len
    line1_1, = axs[0,0].plot(xs, ys)
    line1_2, = axs[0,0].plot(xs, ys)
    line1_3, = axs[0,0].plot(xs, ys)
    line1_4, = axs[0,0].plot(xs, ys)
    line1_5, = axs[0,0].plot(xs, ys)
    line1_6, = axs[0,0].plot(xs, ys)
    line1_7, = axs[0,0].plot(xs, ys)
    line2_1, = axs[0,1].plot(xs, ys)
    line2_2, = axs[0,1].plot(xs, ys)
    line2_3, = axs[0,1].plot(xs, ys)
    line3_1, = axs[1,0].plot(xs, ys)
    line3_2, = axs[1,0].plot(xs, ys)
    line3_3, = axs[1,0].plot(xs, ys)
    line4_1, = axs[1,1].plot(xs, ys)
    line4_2, = axs[1,1].plot(xs, ys)
    line4_3, = axs[1,1].plot(xs, ys)
    line5_1, = axs[2,0].plot(xs, ys)
    line5_2, = axs[2,0].plot(xs, ys)
    line5_3, = axs[2,0].plot(xs, ys)

    axs[0,0].set_xlabel('Sample')
    axs[0,0].set_ylabel('Torques [Nm]')
    axs[0,0].legend(['Joint 1 Torques','Joint 2 Torques','Joint 3 Torques','Joint 4 Torques','Joint 5 Torques','Joint 6 Torques','Joint 7 Torques'], loc='upper center',ncol=4)
    axs[0,0].set_title('Commanded Torques')

    axs[0,1].set_xlabel('Sample')
    axs[0,1].set_ylabel('Force [N]')
    axs[0,1].legend(['X Force', 'Y Force', 'Z Force'])
    axs[0,1].set_title('World Sensed Forces')

    axs[1,0].set_xlabel('Sample')
    axs[1,0].set_ylabel('Position [m]')
    axs[1,0].legend(['X Position', 'Y Position', 'Z Position'])
    axs[1,0].set_title('End-effector position')

    axs[1,1].set_xlabel('Sample')
    axs[1,1].set_ylabel('Force [N]')
    axs[1,1].legend(['X Force', 'Y Force', 'Z Force'])
    axs[1,1].set_title('Ee Sensed Forces')

    axs[2,0].set_xlabel('Sample')
    axs[2,0].set_ylabel('Force [N]')
    axs[2,0].legend(['X Moment', 'Y Moment', 'Z Moment'])
    axs[2,0].set_title('Ee Sensed Moments')

    Time = [0] * x_len
    Torque1 = [0] * x_len
    Torque2 = [0] * x_len
    Torque3 = [0] * x_len
    Torque4 = [0] * x_len
    Torque5 = [0] * x_len
    Torque6 = [0] * x_len
    Torque7 = [0] * x_len
    XForce = [0] * x_len
    YForce = [0] * x_len
    ZForce = [0] * x_len
    XPosition = [0] * x_len
    YPosition = [0] * x_len
    ZPosition = [0] * x_len
    XForceEe = [0] * x_len
    YForceEe = [0] * x_len
    ZForceEe = [0] * x_len
    XMomentsEe = [0] * x_len
    YMomentsEe = [0] * x_len
    ZMomentsEe = [0] * x_len


    r = redis.Redis(host='localhost', port=6379, db=0)

    r.set("Ready_Signal", "False")
    
    while ast.literal_eval(r.get("Ready_Signal").decode('utf-8')) == False:
        if ast.literal_eval(r.get("Ready_Signal").decode('utf-8')) == True:
            break
    
    ani = animation.FuncAnimation(fig,animate,fargs=(Time, Torque1, Torque2, Torque3, Torque4, Torque5, Torque6, Torque7, XForce, YForce, ZForce, 
                                                     XPosition, YPosition, ZPosition, XForceEe, YForceEe, ZForceEe, XMomentsEe, YMomentsEe, ZMomentsEe, r),interval=1, blit=True)

    plt.show()