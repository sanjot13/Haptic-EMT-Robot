/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

// some standard library includes
#include <math.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <cstdlib>

// sai2 main libraries includes
#include "Sai2Model.h" 
#include "Sai2Primitives.h"

// sai2 utilities from sai2-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// namespaces for compactness of code
using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }

// config file names and object names
const string robot_file = "./resources/GEN3_URDF_V12.urdf";

// States 
enum State {
	POSTURE = 0, 
	MOTION
};

// initialize variable to store forces in ee frame and world frame
Vector3d ee_sensed_force = Vector3d(0, 0, 0);
Vector3d world_sensed_force = Vector3d(0, 0, 0);

// function to power numbers 
int pow(double num){
	return num*num;
}

// bool flag_simulation = true;
bool flag_simulation = false;

int main(int argc, char** argv) {
	
	// TESTING --> trying to open python code from controller
	// system("gnome-terminal --command='python3 PlotCommandedTorques.py'");
	// system("code --new-window python3 PlotCommandedTorques.py");

	
	// check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{HW_NUMBER}-control {QUESTION_NUMBER}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 4) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

	// TESTING --> set up different set of keys if we are connected to actual kinova robot
	if(!flag_simulation) {
		// const std::string JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot_command_torques::PANDA";
		// const std::string GRAVITY_COMP_ENABLED_KEY = "cs225a::simviz::gravity_comp_enabled";
		// const std::string Position = "Desired_Position";
		// const std::string POSITION_KEY = "cs225a::simviz::position";
		// const std::string ORIENTATION_KEY = "cs225a::simviz::orientation";
		// const std::string CONTROLLER_READY_KEY = "Controller_Ready";
	}

	// initial state 
	int state = POSTURE;
	string controller_status = "1";

	// start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

	// set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

	// initialize bool ready signal which is used to tell Kinova that controller is connected
	bool ready_signal = true;
	// set controller ready signal to true
	redis_client.setBool(CONTROLLER_READY_KEY,ready_signal);


	// wait for a few seconds until it stops to move, allows time for the kinova to send over its joint positions and velocities
	int t_counter = 0;
	float wait_duration = 1000.0f;
	while (t_counter < (wait_duration)) {
		t_counter++;
		cout<< "wait few secs : " << t_counter   << endl;
	}
	cout<< "wait few secs" << endl;

    // load robots, read current state and update the model
    auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
    robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
    robot->updateModel();

	// send 0 stuff to redis so force plotter can initialize
	redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);
	redis_client.setEigen(EE_FRAME_SENSED_FORCE_KEY, ee_sensed_force);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0, 0.0305, -0.03);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	// pose_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	pose_task->setPosControlGains(100, 20, 0);
	pose_task->setOriControlGains(150, 15, 0);
	
	// initialize variables to store ee position and orientation
	Vector3d ee_pos;
	Matrix3d ee_ori;

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	// joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	joint_task->setGains(100, 20, 0);

	// specify desired posture
	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 15, 0, 120, 0, 45, 67;
	// q_desired.head(7) << 0, 25, 8, 106, 0, 47, 67; // new home 2 position
	// q_desired.head(7) << 0, 35, 8, 106, 0, 38, 67; // new home position
	// q_desired.head(7) << 0, 15, 180, -130, 0, 45, 90; // home position
	// q_desired.head(7) << 0, 0, 180, -90, 0, -90, 90; // tool facing down position
	q_desired.head(7) *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

	// specify desired orientation
	MatrixXd desired_orientation = MatrixXd::Identity(3, 3);
	desired_orientation << 0,cos(M_PI/6),sin(M_PI/6),
						   -1,0,0,
						   0,-sin(M_PI/6),cos(M_PI/6);

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

    // initialize desired task position, quaternion and rotation
    Vector3d desired_task_pos = Vector3d(0, 0, 0);
	VectorXd haply_quat(4);
	haply_quat << 0,0,0,0;
	MatrixXd desired_rotation = MatrixXd::Identity(3,3);

	// set ready signal for plotting to true
	redis_client.set("Ready_Signal", "True");
	
	// set desired end-effector position to zero
	redis_client.setEigen(DESIRED_POSITION_KEY, desired_task_pos);

	// initialize scaling factor
	float scaling_factor = 0.2;
	
    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

		// send time to redis for plotting
		redis_client.set("Time", std::to_string(time));

        // update robot
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		// bounded inertia estimate
		// Eigen::MatrixXd M_updated = robot->M();
		// for (int i=4; i<7; ++i) {
		// 	if (M_updated(i,i) < 0.05) {
		// 		M_updated(i,i) = 0.05;
		// 	}
		// }
		// cout << "robot distal mass: " << M_updated(4,4) << " " << M_updated(5,5) << " " << M_updated(6,6) << " " << endl;
        // robot->updateModel(M_updated);

		// get force in ee frame from sim or bota sensor
		ee_sensed_force = redis_client.getEigen(EE_FRAME_SENSED_FORCE_KEY);
		// converts forces in ee frame to the world frame
		world_sensed_force = robot->rotationInWorld(control_link)*ee_sensed_force;
		// send force in world frame to redis
		redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);

		// std::cout << robot->q().transpose() << "\n";

		if(controller_number = 1){
			// this if case makes the manipulator move to the desired joint positions
			if (state == POSTURE) {
				// update task model 
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);

				command_torques = joint_task->computeTorques() + robot->jointGravityVector();

				// once the manipulator achieves desired joint positions switches to task space control of the end effector
				if ((robot->q() - q_desired).norm() < 2.5e-1) {
					cout << "Posture To Motion" << endl;
					pose_task->reInitializeTask();
					joint_task->reInitializeTask();

					ee_pos = robot->position(control_link, control_point);
					ee_ori = robot->rotation(control_link);

					state = MOTION;
				}
			// this if case controls the end effector position in task space
			} else if (state == MOTION) {
				// obtain desired postion from redis which is usually updated by Haply inverse3
				desired_task_pos = redis_client.getEigen(DESIRED_POSITION_KEY);
				
				// send desired end effector position to redis
				redis_client.setEigen(ACTUAL_DESIRED_POSITION_KEY, ee_pos + desired_task_pos);

				// sets desired position of the end effector
				pose_task->setGoalPosition(ee_pos + desired_task_pos);
				// sets desired orientation of the end effector
				pose_task->setGoalOrientation(desired_orientation);
				
				// TESTING --> was used to control the orientation of the end-effector with the Haply handle
				// haply_quat = redis_client.getEigen("Desired_Quaternion");
				// Eigen::Quaternion desired_quat(haply_quat[0],haply_quat[1],haply_quat[2],haply_quat[3]);
				// desired_rotation = desired_quat.normalized().toRotationMatrix();
				// pose_task->setGoalOrientation(desired_rotation * ee_ori);

				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// cout << robot->position(control_link, control_point)-ee_pos << endl;
				
				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// if case to check if ee has reached end-effector position
				if ((robot->position(control_link, control_point)-ee_pos-desired_task_pos).norm() < 1e-2) {
					cout << "Desired end-effector position reached" << endl;
				}

				// // if case to set scaling factor and send to redis y = 20x - 0.3
				// if (abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) < 0.025) {
				// 	scaling_factor = 0.2;
				// 	redis_client.setDouble(SCALING_KEY, scaling_factor);
				// }
				// else if (abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2) >= 0.025) 
				// 			& abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) < 0.035){
				// 	scaling_factor = 20*abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) - 0.3;
				// 	redis_client.setDouble(SCALING_KEY, scaling_factor);
				// }
				// else if (abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) >= 0.035) {
				// 	scaling_factor = 0.4;
				// 	redis_client.setDouble(SCALING_KEY, scaling_factor);
				// }

				// if case to set scaling factor and send to redis
				if (abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) < 0.025) {
					scaling_factor = 0.1;
					redis_client.setDouble(SCALING_KEY, scaling_factor);
				}
				else if (abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2) >= 0.025) 
							& abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) < 0.035){
					scaling_factor = 30*abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) - 0.65;
					redis_client.setDouble(SCALING_KEY, scaling_factor);
				}
				else if (abs(robot->position(control_link, control_point)(2)-ee_pos(2)-desired_task_pos(2)) >= 0.035) {
					scaling_factor = 0.4;
					redis_client.setDouble(SCALING_KEY, scaling_factor);
				}

			}	 		
		}

		// sending position of end effector in the world frame to redis
		redis_client.setEigen(POSITION_KEY,robot->position(control_link, control_point));

		// saturate command torques if they exceed a certain value, currently set to 35 Nm
		VectorXd abs_command_torques = command_torques.cwiseAbs();
		if (abs_command_torques.maxCoeff() > 35) {
			// cout << abs_command_torques.maxCoeff() << endl;
			float scale = 35/abs_command_torques.maxCoeff();
			command_torques = command_torques*scale;
		}

        // send torques to redis to be obtained by sim or Kinova
        redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY,command_torques);
    }

	// set plotting ready signal to false
	redis_client.set("Ready_Signal", "False");

	// turn off kinova when controller is closed
	ready_signal = false;
	redis_client.setBool(CONTROLLER_READY_KEY,ready_signal);

	// send 0 torques to redis
    command_torques.setZero();
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY,command_torques);

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
