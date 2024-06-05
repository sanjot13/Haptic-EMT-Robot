/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>

#include "utilities.h"
#include <iostream>
#include <fstream>

namespace k_api = Kinova::Api;

#define PORT 10000

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Create an event listener that will set the promise action event to the exit value
// Will set to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
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

bool example_move_to_home_position(k_api::Base::BaseClient* base)
{
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
        if (action.name() == "Home") 
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
        std::promise<k_api::Base::ActionEvent> promise;
        auto future = promise.get_future();
        auto notification_handle = base->OnNotificationActionTopic(
            create_action_event_listener_by_promise(promise),
            k_api::Common::NotificationOptions{}
        );

        base->ExecuteActionFromReference(action_handle);

        // Wait for action to finish
        const auto status = future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }

        return true;

    }
}

bool example_twist_command(k_api::Base::BaseClient* base)
{
    auto command = k_api::Base::TwistCommand();
    command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    command.set_duration(0);  // Unlimited time to execute

    std::cout << "Sending twist command for 5 seconds..." << std::endl;
    auto twist = command.mutable_twist();
    twist->set_linear_x(0.0f);
    twist->set_linear_y(0.03f);
    twist->set_linear_z(0.00f);
    twist->set_angular_x(0.0f);
    twist->set_angular_y(0.0f);
    twist->set_angular_z(5.0f);
    base->SendTwistCommand(command);
    // Let time for twist to be executed
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    std::cout << "Stopping robot ..." << std::endl;
    // Make movement stop
    base->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

bool example_wrench_command(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{

    auto cmd = k_api::Base::WrenchCommand();
    cmd.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    cmd.set_duration(0);  // Unlimited time to execute

    std::cout << "Sending wrench command for 5 seconds..." << std::endl;

    auto wrenchCommand = cmd.mutable_wrench();
    wrenchCommand->set_force_x(0.0f);
    wrenchCommand->set_force_y(0.0f);
    wrenchCommand->set_force_z(10.0f);
    wrenchCommand->set_torque_x(0.0f);
    wrenchCommand->set_torque_z(0.0f);
    wrenchCommand->set_torque_x(0.0f);

    base->SendWrenchCommand(cmd);

    // Write data log to a CSV file
    std::ofstream file;
    file.open("kinova_force_wrenches.txt"); //, std::ios_base::app);

    int timer_count = 0;
    //    int TIME_DURATION = 200.0f; // Duration of the example (seconds)
    int TIME_DURATION = 10.0f; // Duration of the example (seconds)
    while (timer_count < (TIME_DURATION*40)) {
        auto feedback = base_cyclic->RefreshFeedback();
        file << timer_count << "," << feedback.base().tool_external_wrench_force_z() << std::endl;
        std::cout << "tool_external_wrench_force_z " << feedback.base().tool_external_wrench_force_z() << " N" << std::endl;
        timer_count++;
    }

    // Let time for twist to be executed
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    std::cout << "Stopping robot ..." << std::endl;
    // Make movement stop
    base->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);

    // Example core
    bool success = true;
//    success &= example_move_to_home_position(base);
//    success &= example_twist_command(base);

//    success &= example_move_to_home_position(base);
//    success &= example_twist_command(base);

    success &= example_move_to_home_position(base);
    success &= example_wrench_command(base, base_cyclic);

    // Close API session
    session_manager->CloseSession();
    std::cout << "Session closed" << std::endl;

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    return success ? 0: 1;
}