#pragma once

// - read:
const std::string JOINT_ANGLES_KEY = "cs225a::robot_q::PANDA";
const std::string DESIRED_JOINT_ANGLES_KEY = "desired_robot_q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot_dq::PANDA";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot_command_torques::PANDA";
const std::string GRAVITY_COMP_ENABLED_KEY = "cs225a::simviz::gravity_comp_enabled";
const std::string DESIRED_POSITION_KEY = "Desired_Position";
const std::string DESIRED_ORIENTATION_KEY = "Desired_Orientation";
const std::string POSITION_KEY = "cs225a::simviz::position";
const std::string ORIENTATION_KEY = "cs225a::simviz::orientation";
const std::string CONTROLLER_READY_KEY = "Controller_Ready";
const std::string EE_FRAME_SENSED_FORCE_KEY = "ee_Sensed_Force";
const std::string WORLD_FRAME_SENSED_FORCE_KEY = "world_Sensed_Force";
const std::string EE_FRAME_SENSED_MOMENT_KEY = "ee_Sensed_Moment";
const std::string WORLD_FRAME_SENSED_MOMENT_KEY = "world_Sensed_Moment";
const std::string KINOVA_TO_HAPLY_POSITION_KEY = "Kinova_To_Haply_Pos";
const std::string ACTUAL_DESIRED_POSITION_KEY = "Actual_Desired_Position";
const std::string SCALING_KEY = "Scaling";

