/**
 * @file controller.cpp
 * @brief This controller is used to test joint control of the robot.
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

// ------------------------------------------------------- SET UP ---------------------------------------------------------

// specify if stethoscope is attached
bool stethoscope = true;

// specify if only gravity compensation is desired
bool only_gravity_compensation = false;

// ------------------------------------------------------------------------------------------------------------------------

// States 
enum State {
	POSTURE = 0, 
	MOTION
};

// initialize variable to store forces in ee frame and world frame
Vector3d ee_sensed_force = Vector3d(0, 0, 0);
Vector3d world_sensed_force = Vector3d(0, 0, 0);

// bool flag_simulation = true;
bool flag_simulation = false;

int main() {

	// config file names and object names
	string robot_file;
	if (!stethoscope) {
		robot_file = "./resources/GEN3_URDF_V12.urdf";
	} else {
		robot_file = "./resources/GEN3_URDF_V12_stethoscope.urdf";
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

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	// joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING); // this has to be activated for sim to work
	// TESTING -> testing out different BIE thresholds
	joint_task->setBoundedInertiaEstimateThreshold(0.15);
	joint_task->setGains(100, 20, 0);

	// specify desired posture
	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 15, 0, 120, 0, 45, 67; // higher new home 2 position
	// q_desired.head(7) << 0, 20, 0, 130, 0, 30, 67; // new home 2 position
	// q_desired.head(7) << 0, 35, 8, 106, 0, 38, 67; // new home position
	// q_desired.head(7) << 0, 15, 180, -130, 0, 45, 90; // home position
	q_desired.head(7) *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

	// specify desired orientation
	MatrixXd desired_orientation = MatrixXd::Identity(3, 3);
	desired_orientation << 1,0,0,
						   0,1,0,
						   0,0,1;

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);
	timer.enableOvertimeMonitoring(0.1,1,50,true);

    // initialize desired task position, quaternion and rotation
    Vector3d desired_task_pos = Vector3d(0, 0, 0);
	VectorXd haply_quat(4);
	haply_quat << 0,0,0,0;
	MatrixXd desired_rotation = MatrixXd::Identity(3,3);

	// set ready signal for plotting to true
	redis_client.set("Ready_Signal", "True");
	
	// set desired end-effector position to zero
	redis_client.setEigen(DESIRED_POSITION_KEY, desired_task_pos);

	// TESTING->changing gravity vector
	Vector3d gravity = Vector3d(0,0,-9.81);
	robot->setWorldGravity(gravity);

	// set controller ready signal to true
	redis_client.setBool(CONTROLLER_READY_KEY,ready_signal);
	
    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

		// DEBUGGING -> checking mass matrix of kinova
		// cout << robot->M() << '\n' << endl;

		// send time to redis for plotting
		redis_client.set("Time", std::to_string(time));

        // update robot
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();
	
		// send force in world frame to redis
		redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);

		// std::cout << robot->q().transpose() << "\n";

		// this if case makes the manipulator move to the desired joint positions
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			
			if (!only_gravity_compensation){
				command_torques = joint_task->computeTorques() + robot->jointGravityVector();
			} else {
				command_torques = 0*joint_task->computeTorques() + robot->jointGravityVector();
			}

			}

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
