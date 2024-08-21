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
    value6_keys = r.keys('*')
    value7_keys = r.keys('*')
    value8_keys = r.keys('*')
    time_keys = r.keys('*')

    # Filter value keys by name
    filtered_value1_keys = [key for key in value1_keys if key.decode('latin-1') == value_key_name[0]]
    filtered_value2_keys = [key for key in value2_keys if key.decode('latin-1') == value_key_name[1]]
    filtered_value3_keys = [key for key in value3_keys if key.decode('latin-1') == value_key_name[2]]
    filtered_value4_keys = [key for key in value4_keys if key.decode('latin-1') == value_key_name[3]]
    filtered_value5_keys = [key for key in value5_keys if key.decode('latin-1') == value_key_name[4]]
    filtered_value6_keys = [key for key in value6_keys if key.decode('latin-1') == value_key_name[5]]
    filtered_value7_keys = [key for key in value7_keys if key.decode('latin-1') == value_key_name[6]]
    filtered_value8_keys = [key for key in value8_keys if key.decode('latin-1') == value_key_name[7]]
    filtered_time_keys = [key for key in time_keys if key.decode('latin-1') == time_key_name]

    # Use a pipeline to retrieve values for selected keys
    with r.pipeline() as pipe:
        for value1_key, value2_key, value3_key, value4_key, value5_key, value6_key, value7_key, value8_key, time_key in zip(filtered_value1_keys, filtered_value2_keys, filtered_value3_keys, 
                                                                            filtered_value4_keys, filtered_value5_keys, filtered_value6_keys, filtered_value7_keys, filtered_value8_keys, filtered_time_keys):
            pipe.get(value1_key)
            pipe.get(value2_key)
            pipe.get(value3_key)
            pipe.get(value4_key)
            pipe.get(value5_key)
            pipe.get(value6_key)
            pipe.get(value7_key)
            pipe.get(value8_key)
            pipe.get(time_key)
        AllValues = pipe.execute()
    
    # Convert string representation of list to actual list of floats
    values1 = [ast.literal_eval(value1.decode('utf-8')) if value1 else None for value1 in AllValues[::9]]
    values2 = [ast.literal_eval(value2.decode('utf-8')) if value2 else None for value2 in AllValues[1::9]]
    values3 = [ast.literal_eval(value3.decode('utf-8')) if value3 else None for value3 in AllValues[2::9]]
    values4 = [ast.literal_eval(value4.decode('utf-8')) if value4 else None for value4 in AllValues[3::9]]
    values5 = [ast.literal_eval(value5.decode('utf-8')) if value5 else None for value5 in AllValues[4::9]]
    values6 = [ast.literal_eval(value6.decode('utf-8')) if value6 else None for value6 in AllValues[5::9]]
    values7 = [ast.literal_eval(value6.decode('utf-8')) if value6 else None for value6 in AllValues[6::9]]
    values8 = [ast.literal_eval(value6.decode('utf-8')) if value6 else None for value6 in AllValues[7::9]]
    times = [ast.literal_eval(time.decode('utf-8')) if time else None for time in AllValues[8::9]]
    
    # Flatten the lists if necessary
    flat_values1 = [item for sublist in values1 for item in sublist] if values1 else None
    flat_values2 = [item for sublist in values2 for item in sublist] if values2 else None
    flat_values3 = [item for sublist in values3 for item in sublist] if values3 else None
    flat_values4 = [item for sublist in values4 for item in sublist] if values4 else None
    flat_values5 = [item for sublist in values5 for item in sublist] if values5 else None
    flat_values6 = [item for sublist in values6 for item in sublist] if values6 else None
    flat_values7 = [item for sublist in values7 for item in sublist] if values7 else None
    flat_values8 = [item for sublist in values8 for item in sublist] if values8 else None

    # Convert the lists of values and times to NumPy arrays
    values1_array = np.array(flat_values1)
    values2_array = np.array(flat_values2)
    values3_array = np.array(flat_values3)
    values4_array = np.array(flat_values4)
    values5_array = np.array(flat_values5)
    values6_array = np.array(flat_values6)
    values7_array = np.array(flat_values7)
    values8_array = np.array(flat_values8)
    times_array = np.array(times)

    return values1_array, values2_array, values3_array, values4_array, values5_array, values6_array, values7_array, values8_array, times_array

def animate(i,Time, Torque1, Torque2, Torque3, Torque4, Torque5, Torque6, Torque7, SensedTorque1, SensedTorque2, SensedTorque3, SensedTorque4, SensedTorque5, SensedTorque6, SensedTorque7, XForce, YForce, ZForce, 
            XPosition, XPositionDesired, YPosition, YPositionDesired, ZPosition, ZPositionDesired, XForceEe, YForceEe, ZForceEe, XMoments, YMoments, ZMoments, SensedCurrent1, SensedCurrent2, SensedCurrent3, SensedCurrent4, SensedCurrent5, SensedCurrent6, SensedCurrent7, r):
    # Used to calculate loop time
    # time1 = t.time()

    TorqueValue, WorldForceValue, PositionValue, DesiredPositionValue, EeForceValue, WorldMomentValue, SensedTorques, SensedCurrents, TimeValue = redisCatchTorqueAndTime(r, 
                                                                                                   value_key_name=['cs225a::robot_command_torques::PANDA','world_Sensed_Force','cs225a::simviz::position', 'Desired_Position', 'ee_Sensed_Force', 'world_Sensed_Moment', 'Sensed_Torques', 'Sensed_Currents'], 
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
    SensedTorque1.pop(0)
    SensedTorque1.append(-SensedTorques[0])
    SensedTorque2.pop(0)
    SensedTorque2.append(-SensedTorques[1])
    SensedTorque3.pop(0)
    SensedTorque3.append(-SensedTorques[2])
    SensedTorque4.pop(0)
    SensedTorque4.append(-SensedTorques[3])
    SensedTorque5.pop(0)
    SensedTorque5.append(-SensedTorques[4])
    SensedTorque6.pop(0)
    SensedTorque6.append(-SensedTorques[5])
    SensedTorque7.pop(0)
    SensedTorque7.append(-SensedTorques[6])

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

    XPositionDesired.pop(0)
    XPositionDesired.append(DesiredPositionValue[0])
    YPositionDesired.pop(0)
    YPositionDesired.append(DesiredPositionValue[1])
    ZPositionDesired.pop(0)
    ZPositionDesired.append(DesiredPositionValue[2])

    XForceEe.pop(0)
    XForceEe.append(EeForceValue[0])
    YForceEe.pop(0)
    YForceEe.append(EeForceValue[1])
    ZForceEe.pop(0)
    ZForceEe.append(EeForceValue[2])

    XMoments.pop(0)
    XMoments.append(WorldMomentValue[0])
    YMoments.pop(0)
    YMoments.append(WorldMomentValue[1])
    ZMoments.pop(0)
    ZMoments.append(WorldMomentValue[2])

    SensedCurrent1.pop(0)
    SensedCurrent1.append(SensedCurrents[0])
    SensedCurrent2.pop(0)
    SensedCurrent2.append(SensedCurrents[1])
    SensedCurrent3.pop(0)
    SensedCurrent3.append(SensedCurrents[2])
    SensedCurrent4.pop(0)
    SensedCurrent4.append(SensedCurrents[3])
    SensedCurrent5.pop(0)
    SensedCurrent5.append(SensedCurrents[4])
    SensedCurrent6.pop(0)
    SensedCurrent6.append(SensedCurrents[5])
    SensedCurrent7.pop(0)
    SensedCurrent7.append(SensedCurrents[6])
    
    line1_1.set_ydata(Torque1)
    line1_2.set_ydata(Torque2)
    line1_3.set_ydata(Torque3)
    line1_4.set_ydata(Torque4)
    line1_5.set_ydata(Torque5)
    line1_6.set_ydata(Torque6)
    line1_7.set_ydata(Torque7)
    line1_8.set_ydata(SensedTorque1)
    line1_9.set_ydata(SensedTorque2)
    line1_10.set_ydata(SensedTorque3)
    line1_11.set_ydata(SensedTorque4)
    line1_12.set_ydata(SensedTorque5)
    line1_13.set_ydata(SensedTorque6)
    line1_14.set_ydata(SensedTorque7)

    line2_1.set_ydata(XForce)
    line2_2.set_ydata(YForce)
    line2_3.set_ydata(ZForce)

    line3_1.set_ydata(XPosition)
    line3_2.set_ydata(YPosition)
    line3_3.set_ydata(ZPosition)
    line3_4.set_ydata(XPositionDesired)
    line3_5.set_ydata(YPositionDesired)
    line3_6.set_ydata(ZPositionDesired)

    line4_1.set_ydata(XForceEe)
    line4_2.set_ydata(YForceEe)
    line4_3.set_ydata(ZForceEe)

    line5_1.set_ydata(XMoments)
    line5_2.set_ydata(YMoments)
    line5_3.set_ydata(ZMoments)

    line6_1.set_ydata(SensedCurrent1)
    line6_2.set_ydata(SensedCurrent2)
    line6_3.set_ydata(SensedCurrent3)
    line6_4.set_ydata(SensedCurrent4)
    line6_5.set_ydata(SensedCurrent5)
    line6_6.set_ydata(SensedCurrent6)
    line6_7.set_ydata(SensedCurrent7)

    # Used to calculate loop time
    # time2 = t.time()
    # print(time2-time1)

    # print(Time)

    return line1_1, line1_2, line1_3, line1_4, line1_5, line1_6, line1_7, line1_8, line1_9, line1_10, line1_11, line1_12, line1_13, line1_14, line2_1, line2_2, line2_3, line3_1, line3_2, line3_3, line3_4, line3_5, line3_6, line4_1, line4_2, line4_3, line5_1, line5_2, line5_3, line6_1, line6_2, line6_3, line6_4, line6_5, line6_6, line6_7

if __name__ == "__main__":
    fig, axs = plt.subplots(3,2)
    plt.tight_layout()
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    x_len = 400
    axs[0,0].set_ylim([-40,40]) # set ranges for joint torques
    axs[0,1].set_ylim([-5,7.5]) # set ranges for forces in world frame
    axs[1,0].set_ylim([-0.5,0.5]) # set ranges for ee position in world frame
    axs[1,1].set_ylim([-5,7.5])
    axs[2,0].set_ylim([-1,1]) # set ranges for ee sensed moments in world frame
    axs[2,1].set_ylim([-5,5])

    xs = list(range(0, x_len))
    ys = [0] * x_len
    line1_1, = axs[0,0].plot(xs, ys, color='b', alpha=0.5)
    line1_2, = axs[0,0].plot(xs, ys, color='g', alpha=0.5)
    line1_3, = axs[0,0].plot(xs, ys, color='r', alpha=0.5)
    line1_4, = axs[0,0].plot(xs, ys, color='c', alpha=0.5)
    line1_5, = axs[0,0].plot(xs, ys, color='m', alpha=0.5)
    line1_6, = axs[0,0].plot(xs, ys, color='y', alpha=0.5)
    line1_7, = axs[0,0].plot(xs, ys, color='k', alpha=0.5)
    line1_8, = axs[0,0].plot(xs, ys, color='b', linestyle=':')
    line1_9, = axs[0,0].plot(xs, ys, color='g', linestyle=':')
    line1_10, = axs[0,0].plot(xs, ys, color='r', linestyle=':')
    line1_11, = axs[0,0].plot(xs, ys, color='c', linestyle=':')
    line1_12, = axs[0,0].plot(xs, ys, color='m', linestyle=':')
    line1_13, = axs[0,0].plot(xs, ys, color='y', linestyle=':')
    line1_14, = axs[0,0].plot(xs, ys, color='k', linestyle=':')
    line2_1, = axs[0,1].plot(xs, ys)
    line2_2, = axs[0,1].plot(xs, ys)
    line2_3, = axs[0,1].plot(xs, ys)
    line3_1, = axs[1,0].plot(xs, ys, color='r', alpha=0.5)
    line3_2, = axs[1,0].plot(xs, ys, color='b', alpha=0.5)
    line3_3, = axs[1,0].plot(xs, ys, color='g', alpha=0.5)
    line3_4, = axs[1,0].plot(xs, ys, color='r', linestyle=':')
    line3_5, = axs[1,0].plot(xs, ys, color='b', linestyle=':')
    line3_6, = axs[1,0].plot(xs, ys, color='g', linestyle=':')
    line4_1, = axs[1,1].plot(xs, ys)
    line4_2, = axs[1,1].plot(xs, ys)
    line4_3, = axs[1,1].plot(xs, ys)
    line5_1, = axs[2,0].plot(xs, ys)
    line5_2, = axs[2,0].plot(xs, ys)
    line5_3, = axs[2,0].plot(xs, ys)
    line6_1, = axs[2,1].plot(xs, ys)
    line6_2, = axs[2,1].plot(xs, ys)
    line6_3, = axs[2,1].plot(xs, ys)
    line6_4, = axs[2,1].plot(xs, ys)
    line6_5, = axs[2,1].plot(xs, ys)
    line6_6, = axs[2,1].plot(xs, ys)
    line6_7, = axs[2,1].plot(xs, ys)

    axs[0,0].set_xlabel('Sample')
    axs[0,0].set_ylabel('Torques [Nm]')
    axs[0,0].legend(['Torq 1', 'Torq 2', 'Torq 3', 'Torq 4', 'Torq 5', 'Torq 6', 'Torq 7', 'S Torq 1', 'S Torq 2', 'S Torq 3', 'S Torq 4', 'S Torq 5', 'S Torq 6', 'S Torq 7'], loc='upper center',ncol=7, columnspacing=0.9, fontsize=9.0)
    axs[0,0].set_title('Commanded Torques')

    axs[0,1].set_xlabel('Sample')
    axs[0,1].set_ylabel('Force [N]')
    axs[0,1].legend(['X Force', 'Y Force', 'Z Force'])
    axs[0,1].set_title('World Sensed Forces')

    axs[1,0].set_xlabel('Sample')
    axs[1,0].set_ylabel('Position [m]')
    axs[1,0].legend(['X Pos', 'Y Pos', 'Z Pos', 'Desired X Pos', 'Desired Y Pos', 'Desired Z Pos'], loc='upper center',ncol=6, columnspacing=1)
    axs[1,0].set_title('End-effector position')

    axs[1,1].set_xlabel('Sample')
    axs[1,1].set_ylabel('Force [N]')
    axs[1,1].legend(['X Force', 'Y Force', 'Z Force'])
    axs[1,1].set_title('Ee Sensed Forces')

    axs[2,0].set_xlabel('Sample')
    axs[2,0].set_ylabel('Moment [Nm]')
    axs[2,0].legend(['X Moment', 'Y Moment', 'Z Moment'])
    axs[2,0].set_title('World Sensed Moments')

    axs[2,1].set_xlabel('Sample')
    axs[2,1].set_ylabel('Current [A]')
    axs[2,1].legend(['Current 1', 'Current 2', 'Current 3', 'Current 4', 'Current 5', 'Current 6', 'Current 7'])
    axs[2,1].set_title('Sensed Currents')

    Time = [0] * x_len
    Torque1 = [0] * x_len
    Torque2 = [0] * x_len
    Torque3 = [0] * x_len
    Torque4 = [0] * x_len
    Torque5 = [0] * x_len
    Torque6 = [0] * x_len
    Torque7 = [0] * x_len
    SensedTorque1 = [0] * x_len
    SensedTorque2 = [0] * x_len
    SensedTorque3 = [0] * x_len
    SensedTorque4 = [0] * x_len
    SensedTorque5 = [0] * x_len
    SensedTorque6 = [0] * x_len
    SensedTorque7 = [0] * x_len
    XForce = [0] * x_len
    YForce = [0] * x_len
    ZForce = [0] * x_len
    XPosition = [0] * x_len
    XPositionDesired = [0] * x_len
    YPosition = [0] * x_len
    YPositionDesired = [0] * x_len
    ZPosition = [0] * x_len
    ZPositionDesired = [0] * x_len
    XForceEe = [0] * x_len
    YForceEe = [0] * x_len
    ZForceEe = [0] * x_len
    XMoments = [0] * x_len
    YMoments = [0] * x_len
    ZMoments = [0] * x_len
    SensedCurrent1 = [0] * x_len
    SensedCurrent2 = [0] * x_len
    SensedCurrent3 = [0] * x_len
    SensedCurrent4 = [0] * x_len
    SensedCurrent5 = [0] * x_len
    SensedCurrent6 = [0] * x_len
    SensedCurrent7 = [0] * x_len

    r = redis.Redis(host='localhost', port=6379, db=0)

    r.set("Ready_Signal", "False")
    
    while ast.literal_eval(r.get("Ready_Signal").decode('utf-8')) == False:
        if ast.literal_eval(r.get("Ready_Signal").decode('utf-8')) == True:
            break
    
    ani = animation.FuncAnimation(fig,animate,fargs=(Time, Torque1, Torque2, Torque3, Torque4, Torque5, Torque6, Torque7, SensedTorque1, SensedTorque2, SensedTorque3, SensedTorque4, SensedTorque5, SensedTorque6, SensedTorque7, XForce, YForce, ZForce, 
            XPosition, XPositionDesired, YPosition, YPositionDesired, ZPosition, ZPositionDesired, XForceEe, YForceEe, ZForceEe, XMoments, YMoments, ZMoments, SensedCurrent1, SensedCurrent2, SensedCurrent3, SensedCurrent4, SensedCurrent5, SensedCurrent6, SensedCurrent7, r),interval=1, blit=True)

    plt.show()