/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

/*
* DESCRIPTION OF CURRENT EXAMPLE:
* ===============================
* This example works as a simili-haptic demo.
*     
* The last actuator, the small one holding the interconnect, acts as a torque sensing device commanding the first actuator.
* The first actuator, the big one on the base, is controlled in torque and its position is sent as a command to the last one.
* 
* The script can be launched through command line with python3: python torqueControl_example.py
* The PC should be connected through ethernet with the arm. Default IP address 192.168.1.10 is used as arm address.
* 
* 1- Connection with the base:
*     1- A TCP session is started on port 10000 for most API calls. Refresh is at 25ms on this port.
*     2- A UDP session is started on port 10001 for BaseCyclic calls. Refresh is at 1ms on this port only.
* 2- Initialization
*     1- First frame is built based on arm feedback to ensure continuity
*     2- First actuator torque command is set as well
*     3- Base is set in low-level servoing
*     4- First frame is sent
*     3- First actuator is switched to torque mode
* 3- Cyclic thread is running at 1ms
*     1- Torque command to first actuator is set to a multiple of last actuator torque measure minus its initial value to
*        avoid an initial offset error
*     2- Position command to last actuator equals first actuator position minus initial delta
*     
* 4- On keyboard interrupt, example stops
*     1- Cyclic thread is stopped
*     2- First actuator is set back to position control
*     3- Base is set in single level servoing (default)
*/

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include "utilities.h"

#include <google/protobuf/util/json_util.h>

#include <mutex>
#include <thread>

#include <time.h>
#include <fstream>

#include </usr/include/hiredis/hiredis.h>
#include <RedisClient.h>
#include <redis_keys.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif

namespace k_api = Kinova::Api;
using namespace Eigen;

#define PORT 10000
#define PORT_REAL_TIME 10001

float TIME_DURATION = 1200.0f; // Duration of the example (seconds)

// Initialize commanded torques vector
VectorXd command_torques = VectorXd::Zero(7);

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
 
    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

/**************************
 * Example core functions *
 **************************/
 // function to move the Kinova to home position
bool example_move_to_home_position(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();

    k_api::BaseCyclic::Feedback base_feedback;

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "New Home 2")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base_feedback = base_cyclic->RefreshFeedback();
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

//        // Wait for a few seconds until it stops to move
//        int t_counter = 0;
//        float wait_duration = 5.0f;
//        while (t_counter < (wait_duration * 200000)) {
//            t_counter++;
//            cout<< "wait few secs : " << t_counter   << endl;
//        }
//        cout<< "wait few secs" << endl;

        // Print out the Home Position
        cout << "Home Position: ";
        for (unsigned int i = 0; i < actuator_count; i++){
            cout << base_feedback.actuators(i).position() << ", ";
        }
        cout << endl;

        return true;
    }
}

// function to set the Kinova to torque control mode
bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config,
                                   Sai2Common::RedisClient* redis_client, bool controller_ready)
{
    // initialize vectors that will be used to store joint positions and velocities later
    VectorXd current_Position(7) ;
    VectorXd current_Velocity(7) ;

    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();
    
    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position()); // ASK WHAT IS THIS FOR

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        
        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        int first_actuator_device_id = 1;
        int second_actuator_device_id = 2;
        int third_actuator_device_id = 3;
        int fourth_actuator_device_id = 4;
        int fifth_actuator_device_id = 5;
        int sixth_actuator_device_id = 6;
        int seventh_actuator_device_id = 7;
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, second_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, third_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, fourth_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, fifth_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, sixth_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, seventh_actuator_device_id);

        // Initial delta between first and last actuator
        float init_delta_position = base_feedback.actuators(0).position() - base_feedback.actuators(actuator_count - 1).position();

        // Initial first and last actuator torques; avoids unexpected movement due to torque offsets
//        float init_first_torque = -base_feedback.actuators(0).torque(); //Torque measure is reversed compared to actuator direction
//        float init_second_torque = -base_feedback.actuators(1).torque(); //Torque measure is reversed compared to actuator direction
//        float init_third_torque = -base_feedback.actuators(2).torque(); //Torque measure is reversed compared to actuator direction
//        float init_fourth_torque = -base_feedback.actuators(3).torque(); //Torque measure is reversed compared to actuator direction
//        float init_fifth_torque = -base_feedback.actuators(4).torque(); //Torque measure is reversed compared to actuator direction
//        float init_sixth_torque = -base_feedback.actuators(5).torque(); //Torque measure is reversed compared to actuator direction
//        float init_seventh_torque = -base_feedback.actuators(6).torque(); //Torque measure is reversed compared to actuator direction


//        float init_torques[actuator_count];
//        for (int i = 0; i < actuator_count; i++){
//            init_torques[i] = -base_feedback.actuators(i).torque();
//            cout<<i<<": "<<init_torques[i]<<endl;
//        }

        float torque_amplification = 2.0;

        std::cout << "Running torque control example for " << TIME_DURATION << " seconds" << std::endl;
        std::ofstream file;
        file.open("kinova_rotation_forces.txt"); //, std::ios_base::app);

        // Real-time loop
//        while (timer_count < (TIME_DURATION * 1000))
        // TESTING --> have this run while controller_ready is set to true
        while (controller_ready)
        {
            now = GetTickUs();

            // TESTING --> continuously check controller_ready if it is turned on
            controller_ready = redis_client->getBool(CONTROLLER_READY_KEY);

            if (now - last > 1)
            {
//                base_feedback.base().tool_external_wrench_force_x()
//                std::cout << "tool_external_wrench_force_x " << base_feedback.base().tool_external_wrench_force_x() << " N" << std::endl;
//                std::cout << "tool_external_wrench_force_y " << base_feedback.base().tool_external_wrench_force_x() << " N" << std::endl;
                file << timer_count << "," << base_feedback.base().tool_external_wrench_force_z() << std::endl;
//                std::cout << "tool_external_wrench_force_z " << base_feedback.base().tool_external_wrench_force_z() << " N" << std::endl;

                // grab torques commanded by controller from redis
                command_torques = redis_client->getEigen(JOINT_TORQUES_COMMANDED_KEY);

                // for loop is to saturate the torques, so they stay within safety bounds (kills function if exceed safety limits)
                for (int i = 0; i < actuator_count; i++){
                    if (abs(command_torques[i]) > 40) {
                        cout << "Killing due to saturation" << endl;
                        controller_ready = false;
                        redis_client->setBool(CONTROLLER_READY_KEY, controller_ready);
                    }
                }

                // Position command to first actuator is set to measured one to avoid following error to trigger
                // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                //        actuator continues to move under torque command, resulting position error with command will
                //        trigger a following error and switch back the actuator in position command to hold its position
                base_command.mutable_actuators(0)->set_position(base_feedback.actuators(0).position());
                base_command.mutable_actuators(1)->set_position(base_feedback.actuators(1).position());
                base_command.mutable_actuators(2)->set_position(base_feedback.actuators(2).position());
                base_command.mutable_actuators(3)->set_position(base_feedback.actuators(3).position());
                base_command.mutable_actuators(4)->set_position(base_feedback.actuators(4).position());
                base_command.mutable_actuators(5)->set_position(base_feedback.actuators(5).position());
                base_command.mutable_actuators(6)->set_position(base_feedback.actuators(6).position());

                // grabs current positions of joints and saves it into a vector
                current_Position << base_feedback.actuators(0).position(),base_feedback.actuators(1).position(),base_feedback.actuators(2).position(),
                                    base_feedback.actuators(3).position(),base_feedback.actuators(4).position(),base_feedback.actuators(5).position(),base_feedback.actuators(6).position();
//                cout << current_Position << endl;
//                cout << "------degrees------------";
                current_Position.head(7) *=  M_PI/180;
//                cout << current_Position << endl;

                // grabs current velocities of joints and saves it into a vector
                current_Velocity << base_feedback.actuators(0).velocity(),base_feedback.actuators(1).velocity(),base_feedback.actuators(2).velocity(),
                        base_feedback.actuators(3).velocity(),base_feedback.actuators(4).velocity(),base_feedback.actuators(5).velocity(),base_feedback.actuators(6).velocity();
                current_Velocity.head(7) *=  M_PI/180;

                // re-center angles from [0, 360] to [-180, 180]
                for (int i = 0; i < 7; ++i) {
                    if (current_Position(i) > M_PI) {
                        current_Position(i) -= 2 * M_PI;
                    }
                }

                // sends joint positions and velocities of kinova to redis
                redis_client->setEigen(JOINT_ANGLES_KEY,current_Position);
                redis_client->setEigen(JOINT_VELOCITIES_KEY,current_Velocity);

                // First actuator torque command is set to last actuator torque measure times an amplification
                /*
                 * fastest response with factor of 10.
                 * (base_feedback.actuators(actuator_count - 1).torque() - 10*init_seventh_torque)
                 *
                 * can try larger scalar instead of 0 + init_first_torque
                 */
//                base_command.mutable_actuators(0)->set_torque_joint(6 + 1*init_first_torque + (torque_amplification * (base_feedback.actuators(actuator_count - 1).torque() - init_seventh_torque)));

//                base_command.mutable_actuators(0)->set_torque_joint(command_torques[0]+init_first_torque); // command first joint
//                base_command.mutable_actuators(0)->set_torque_joint(init_first_torque); // command first joint
//                base_command.mutable_actuators(1)->set_torque_joint(-8.4); // command second joint
//                base_command.mutable_actuators(2)->set_torque_joint(init_third_torque); // command third joint
//                base_command.mutable_actuators(3)->set_torque_joint(init_fourth_torque); // command fourth joint
//                base_command.mutable_actuators(4)->set_torque_joint(init_fifth_torque); // command fifth joint
//                base_command.mutable_actuators(5)->set_torque_joint(init_sixth_torque); // command sixth joint
//                base_command.mutable_actuators(6)->set_torque_joint(init_seventh_torque);        // command seventh joint


//
//                base_command.mutable_actuators(0)->set_torque_joint(.85); // command first joint
//                base_command.mutable_actuators(1)->set_torque_joint(-9.3); // command second joint
//                base_command.mutable_actuators(2)->set_torque_joint(-.35); // command third joint
//                base_command.mutable_actuators(3)->set_torque_joint(3.9); // command fourth joint
//                base_command.mutable_actuators(4)->set_torque_joint(.18); // command fifth joint
//                base_command.mutable_actuators(5)->set_torque_joint(.93); // command sixth joint
//                base_command.mutable_actuators(6)->set_torque_joint(.11);        // command seventh joint
//                float desired_torques[actuator_count] = {0, -0.9, 0, 0, 0, 0, 0};
//                float simulated_torques[actuator_count] = {-0.000115,-9.140213,-0.108413,4.017534,0.007983,0.968525,-0.000980};

                // for loop to set the torques in each joint
                for (int i = 0; i < actuator_count; i++){
                    base_command.mutable_actuators(i)->set_torque_joint(command_torques[i]);
                }

                // Printing actual torque from each joint
//                cout << "Command Torque: ";
//                for (int i = 0; i < actuator_count; i++) {
//                    cout<< command_torques[i]<< ", ";
//                }
//                cout << endl;
//
//                // Printing actual torque from each joint
//                cout << "Actual Torque: ";
//                for (int i = 0; i < actuator_count; i++) {
//                    cout<< base_feedback.actuators(i).torque() << ", ";
//                }
//                cout << endl;

                // Printing initial torque values
//                cout << "Initial Torque: "<<init_first_torque << ',' << init_second_torque -0.6<< ',' << init_third_torque << ',' << init_fourth_torque << ',' << init_fifth_torque << ',' << init_sixth_torque << ',' << init_seventh_torque << endl;

//                // Printing actual torque from each joint
//                cout << "Position: ";
//                for (int i = 0; i < actuator_count; i++) {
//                    cout<< base_feedback.actuators(i).position() << ", ";
//                }
//                cout << endl;


                // First actuator position is sent as a command to last actuator
//                base_command.mutable_actuators(actuator_count - 1)->set_position(base_feedback.actuators(0).position() - init_delta_position);


                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                for (int idx = 0; idx < actuator_count; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }

                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                }
                catch (k_api::KDetailedException& ex)
                {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;

                    std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                }
                catch (std::runtime_error& ex2)
                {
                    std::cout << "runtime error: " << ex2.what() << std::endl;
                }
                catch(...)
                {
                    std::cout << "Unknown error." << std::endl;
                }
                
                timer_count++;
                last = GetTickUs();
            }
        }

        std::cout << "Torque control example exited" << std::endl;

        // Set first actuator back in position 
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, second_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, third_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, fourth_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, fifth_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, sixth_actuator_device_id);
        actuator_config->SetControlMode(control_mode_message, seventh_actuator_device_id);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    
    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

// main code
int main(int argc, char **argv)
{

    // Initialize redis server
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // Set all commanded torques in redis to 0 before proceeding
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY,command_torques);

    // Set controller ready key to false when kinova is first started and send to redis
    bool controller_ready = false;
    redis_client.setBool(CONTROLLER_READY_KEY,controller_ready);

    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Example core
    bool success = true;
    // start by moving arm into home position
    success &= example_move_to_home_position(base, base_cyclic);
    cout << "Waiting for controller" << endl;

    // TESTING --> this was used to set the initial variables before, should not be needed now that controller has a wait timer
    VectorXd starting_position(7);
    starting_position << 0, 20, 4, 130, 0, 30, 67;
    starting_position.array() *= M_PI / 180;
    for (int i = 0; i < 7; ++i) {
        if (starting_position(i) > M_PI) {
            starting_position(i) -= 2 * M_PI;
        }
    }
    redis_client.setEigen(JOINT_ANGLES_KEY, starting_position);
    redis_client.setEigen(JOINT_VELOCITIES_KEY, starting_position * 0);
    std::cout << "Actual starting position: " << starting_position.transpose() << "\n";

    // this forces the Kinova to run torque control only if the controller has been turned on
    while (!controller_ready) {
        controller_ready = redis_client.getBool(CONTROLLER_READY_KEY);
    }
    cout << "Controller turned on" << endl;
    success &= example_cyclic_torque_control(base, base_cyclic, actuator_config, &redis_client, controller_ready);

    // TESTING --> set the arm back to joint control to move it back to home position when torque control is completed
    success &= example_move_to_home_position(base, base_cyclic);

    if (!success)
    {
        std::cout << "There has been an unexpected error." << endl;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}
