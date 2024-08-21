/**
 * @file controller.cpp
 * @brief Has the robot cycle through several end effector orientations
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

// specify saturation values for linear and angular velocity
double linear_velocity_sat = 0.2;
double angular_velocity_sat = M_PI/5;

// specify how long to do rotation cycling
double stop_time = 30;

// ------------------------------------------------------------------------------------------------------------------------

// States 
enum State {
	POSTURE = 0, 
	MOTION,
	ROTATE,
};

// initialize variable to store forces and moments in ee frame and world frame
Vector3d ee_sensed_force = Vector3d(0, 0, 0);
Vector3d world_sensed_force = Vector3d(0, 0, 0);
Vector3d ee_sensed_moment = Vector3d(0, 0, 0); 
Vector3d world_sensed_moment = Vector3d(0, 0, 0);

// bool flag_simulation = true;
bool flag_simulation = false;

int main(int argc, char** argv) {

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
	redis_client.setEigen(WORLD_FRAME_SENSED_MOMENT_KEY, world_sensed_moment);

	// intialize vector for gravity
	Vector3d gravity = Vector3d(0,0,-9.81);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	string control_link;
	Vector3d control_point;
	if (!stethoscope){
		control_link = "end-effector";
		control_point << 0, 0, -0.03;
	} else {
		control_link = "stethoscope";
		control_point << 0, 0, -0.0377;
	}
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	// pose_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING); // this needs to be active for sim to work
	pose_task->setBoundedInertiaEstimateThreshold(0.15);
	pose_task->setPosControlGains(100, 20, 0);
	pose_task->setOriControlGains(100, 20, 0);
	// set the force sensor location for the contact part of the task
	pose_task->setForceSensorFrame(control_link, Affine3d::Identity());
	// set velocity saturation values
	pose_task->enableVelocitySaturation(linear_velocity_sat, angular_velocity_sat);
	
	// initialize desired task position vector
	Vector3d desired_task_pos = Vector3d(0, 0, 0);

	// setting up desired positions 
	Vector3d waypoint1 = Vector3d(0.35, 0, 0.3);

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	// joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING); // this needs to be active for sim to work
	joint_task->setBoundedInertiaEstimateThreshold(0.15);
	joint_task->setGains(100, 20, 0);

	// specify desired posture
	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 15, 0, 120, 0, 45, 67; // higher new home 2 position
	// q_desired.head(7) << 0, 25, 8, 106, 0, 47, 67; // new home 2 position
	// q_desired.head(7) << 0, 35, 8, 106, 0, 38, 67; // new home position
	// q_desired.head(7) << 0, 15, 180, -130, 0, 45, 90; // home position
	q_desired.head(7) *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

	// variables related to cycling through orientations
	// initialize orientation counter
	int orientation_counter = 0;
	// initialize time saved counter
	bool time_saved = false;
	// initialize variable to save time
	double initial_time = 0.0;

	// specify desired orientation
	MatrixXd desired_orientation = MatrixXd::Identity(3, 3);
	// start with flat orientation 
	desired_orientation << 0,1,0,
						   -1,0,0,
						   0,0,1;

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	// intialize desired angular velocity
	Vector3d desired_angular_velocity = Vector3d(0, 0, 0);

	// set ready signal for plotting to true
	redis_client.set("Ready_Signal", "True");
	
	// set desired end-effector position to waypoint 1
	desired_task_pos = waypoint1;
	redis_client.setEigen(DESIRED_POSITION_KEY, desired_task_pos);
	
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

		// get force in ee frame from sim or bota sensor
		ee_sensed_force = redis_client.getEigen(EE_FRAME_SENSED_FORCE_KEY);
		ee_sensed_moment = redis_client.getEigen(EE_FRAME_SENSED_MOMENT_KEY);

		// std::cout << robot->q().transpose() << "\n";

		// update force sensor values (needs to be the force applied by the robot to the environment, in sensor frame)
		ee_sensed_force = ee_sensed_force - pose_task->getCurrentOrientation().transpose()*0.174*gravity;
		// DEBUGGING->priting out calculated gravitational force from attached stethoscope mount
		cout << (pose_task->getCurrentOrientation()*0.174*gravity).transpose() << endl;
		pose_task->updateSensedForceAndMoment(-ee_sensed_force, -ee_sensed_moment);

		// get sensed forces in world frame
		world_sensed_force = -pose_task->getSensedForceControlWorldFrame();
		// send force in world frame to redis
		redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);
		// get sensed moment in world frame
		world_sensed_moment = -pose_task->getSensedMomentControlWorldFrame();
		// send moment in world frame to redis
		redis_client.setEigen(WORLD_FRAME_SENSED_MOMENT_KEY, world_sensed_moment);

		// this if case makes the manipulator move to the desired joint positions
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			command_torques = joint_task->computeTorques() + robot->jointGravityVector();

			// once the manipulator achieves desired joint positions switches to task space control of the end effector
			if ((robot->q() - q_desired).norm() < 0.5e-1) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				state = MOTION;
			}
		// this if case controls the end effector position in task space
		} else if (state == MOTION) {
			// obtain desired postion from redis 
			desired_task_pos = redis_client.getEigen(DESIRED_POSITION_KEY);
			
			// sets desired position of the end effector
			pose_task->setGoalPosition(desired_task_pos);
			// sets desired orientation of the end effector
			pose_task->setGoalOrientation(desired_orientation);

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			// have the joint task in the null space of the pose task
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			// to compute the required torques to accomplish task
			command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();
			
			// Debugging->checking position error and orientation control error
			cout << pose_task->getPositionError().norm() << " " << pose_task->getOrientationError().norm() << endl;

			// cycle through different orientations
			if (pose_task->goalPositionReached(0.015) && pose_task->goalOrientationReached(0.03) && !time_saved && time < stop_time){
				if (orientation_counter == 0){
					// tilt along world y axis
					desired_orientation << 0,cos(M_PI/9),sin(M_PI/9),
										   -1,0,0,
										   0,-sin(M_PI/9),cos(M_PI/9);
					orientation_counter = 1;
				} else if (orientation_counter == 1){
					// tilt along world x axis
						desired_orientation << 0,1,0, 
											   -cos(-M_PI/9),0,-sin(-M_PI/9),
											   -sin(-M_PI/9),0,cos(-M_PI/9);
					orientation_counter = 2;
				} else if (orientation_counter == 2) {
					// flat orientation 
						desired_orientation << 0,1,0,
											-1,0,0,
											0,0,1;
					orientation_counter = 0;
				}
				if (!time_saved){
					initial_time = time;
					time_saved = true;
				}
			}

			if (time_saved & (time-initial_time > 3)){
				time_saved = false;
			}

			// switch to rotate state when we reach desired position and orientation
			// if (pose_task->goalPositionReached(0.015) && pose_task->goalOrientationReached(0.05)){
			// 	state = ROTATE;
			// 	cout << "motion to rotate" << endl;
			// }
		} else if (state == ROTATE){
			
			if (time < 5) {
				desired_angular_velocity << 0, M_PI/5, 0;
				pose_task->setGoalAngularVelocity(desired_angular_velocity);
				pose_task->setGoalOrientation(pose_task->getCurrentOrientation());
			} else {
				desired_angular_velocity << 0, 0, 0;
				pose_task->setGoalAngularVelocity(desired_angular_velocity);
			}

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			// have the joint task in the null space of the pose task
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			// to compute the required torques to accomplish task
			command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

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

		// TESTING->Looking at mass matrix for arm
		// cout << robot->M() << endl;

		// cout << desired_angular_velocity.transpose() << endl;

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
