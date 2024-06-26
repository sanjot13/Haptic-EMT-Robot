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
#include <fstream>

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
	MOTION,
	DROP,
	FORCE,
	RESET
};

// initialize variable to store forces and moments in ee frame and world frame
Vector3d ee_sensed_force = Vector3d(0, 0, 0);
Vector3d world_sensed_force = Vector3d(0, 0, 0);
Vector3d ee_sensed_moment = Vector3d(0, 0, 0); // not used, set to 0 for now

// function to power numbers 
int pow(double num){
	return num*num;
}

// bool flag_simulation = true;
bool flag_simulation = false;

// setting up variable to keep track of locations
int location_counter = 0;

// setting variable to keep track if an inital time has been saved
bool time_saved = false;

// setting up variable to save initial time we reach desired ee position
double initial_time = 0;

// setting up desired positions 
Vector3d waypoint1 = Vector3d(0.3, 0, 0.2);
Vector3d waypoint2 = Vector3d(0.3, -0.14, 0.2);
Vector3d waypoint3 = Vector3d(0.37, 0.19, 0.2);

int main(int argc, char** argv) {
	
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

	// initialize counter to keep track of force readings
	int force_reading_counter = 0;
	// vector to store desired force readings to go through
	// VectorXd desired_forces {{-3, -4, -5, -6, -7}};
	VectorXd desired_forces {{-2, -3}}; 
	// initialize vector to keep track of previous world force
	VectorXd previous_world_sensed_z_forces = VectorXd::Zero(12);
	previous_world_sensed_z_forces[11] = 10;	
	std::cout << previous_world_sensed_z_forces << endl;
	// setting up variable to keep count of number of loop iterations
	int loop_counter = 1;

	// Open a file to write force readings
    std::ofstream outputFile("readings.txt");

    // Check if the file is opened successfully
    if (!outputFile.is_open()) {
        cout << "File was not opened" << endl;
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
	const Vector3d control_point = Vector3d(0, 0.0305, -0.03); // current control point is set to edge of ee
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	// pose_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING); // sim needs this to be activated instead to work
	pose_task->setPosControlGains(100, 20, 0);
	pose_task->setOriControlGains(175, 27, 0);
	// set the force sensor location for the contact part of the task
	pose_task->setForceSensorFrame(control_link, Affine3d::Identity());
	
	// initialize variables to store ee position and orientation
	Vector3d ee_pos;
	Matrix3d ee_ori;

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	// joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING); // sim needs this to be activated to work
	// TESTING->setting up some integral gains on joints 1 and 3
	// VectorXd kp_gains {{100, 100, 100, 100, 100, 100, 100}};
	// VectorXd kv_gains {{20, 20, 20, 20, 20, 20, 20}};
	// VectorXd ki_gains {{5, 0, 5, 0, 0, 0, 0}};
	// joint_task->setGains(kp_gains, kv_gains, ki_gains); 
	joint_task->setGains(100,20,0);

	// specify desired posture
	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 20, 0, 130, 0, 30, 67;
	q_desired.head(7) *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

	// specify desired orientation
	MatrixXd desired_orientation = MatrixXd::Identity(3, 3);
	// pointed orientation
	desired_orientation << 0,cos(M_PI/6),sin(M_PI/6),
						   -1,0,0,
						   0,-sin(M_PI/6),cos(M_PI/6);

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

    // initialize desired task position, quaternion and rotation
    // Vector3d desired_task_pos = Vector3d(0, 0, -0.03);
	Vector3d desired_task_pos = Vector3d(0, 0, 0);
	Vector3d force_desired_task_pos = Vector3d(0, 0, 0);
	VectorXd haply_quat(4);
	haply_quat << 0,0,0,0;
	MatrixXd desired_rotation = MatrixXd::Identity(3,3);

	// set ready signal for plotting to true
	redis_client.set("Ready_Signal", "True");
	
	// set desired end-effector position to zero
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

		// std::cout << robot->q().transpose() << "\n";

		// update force sensor values (needs to be the force applied by the robot to the environment, in sensor frame)
		pose_task->updateSensedForceAndMoment(-ee_sensed_force, ee_sensed_moment);

		// converts forces in ee frame to the world frame
		// world_sensed_force = robot->rotationInWorld(control_link)*ee_sensed_force;
		world_sensed_force = -pose_task->getSensedForceControlWorldFrame();
		// send force in world frame to redis
		redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);

		if(controller_number = 1){
			// this if case makes the manipulator move to the desired joint positions
			if (state == POSTURE) {
				// update task model 
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);

				command_torques = joint_task->computeTorques() + robot->jointGravityVector();

				// once the manipulator achieves desired joint positions switches to task space control of the end effector
				// if ((robot->q() - q_desired).norm() < 1e-2) {
				if ((robot->q() - q_desired).norm() < 2.5e-1) {
					cout << "Posture To Motion" << endl;
					pose_task->reInitializeTask();
					joint_task->reInitializeTask();

					ee_pos = robot->position(control_link, control_point);
					ee_ori = robot->rotation(control_link);

					// set desired task pos variable to first waypoint
					desired_task_pos = waypoint1;
					redis_client.setEigen(DESIRED_POSITION_KEY,desired_task_pos);

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

				// sets desired position of joints
				joint_task->setGoalPosition(q_desired);

				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// Debugging->checking position error and orientation control error
				cout << pose_task->getPositionError().norm() << " " << pose_task->getOrientationError().norm() << endl;

				if (pose_task->goalPositionReached(0.015) && pose_task->goalOrientationReached(0.05)){
					// save the time we reach the goal position
					if (time_saved == false) {
						cout << "goal position reached" << endl;
						time_saved = true;
						initial_time = time;
						cout << initial_time << endl;
					}

					// switch to force control after 3 seconds, second condition is to prevent it from repeating the last force measurement
					if (time - initial_time > 3) {
						if (location_counter < 3) {
						outputFile << "Reading " << location_counter+1 << endl;
						
						// pose_task->enableVelocitySaturation(0.01);
				
						// setting desired postion to z = 0
						// desired_task_pos[2] = 0;
						// redis_client.setEigen(DESIRED_POSITION_KEY,desired_task_pos);
						// pose_task->setGoalPosition(desired_task_pos);

						// swap to drop state
						state = DROP;
						cout << "motion to drop" << endl;
						} else {
							runloop = false;
						}
					} 
				}
			} else if (state == DROP){
				// manually changing desired position to lower ee at a speed of 0.07 m/s
				desired_task_pos[2] -= 0.000007;
				redis_client.setEigen(DESIRED_POSITION_KEY,desired_task_pos);
				pose_task->setGoalPosition(desired_task_pos);

				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// swap to force state if a force is detected at ee
				if (pose_task->getSensedForceControlWorldFrame()(2) <= -1.0){

					// pose_task->disableVelocitySaturation();
					
					cout << "drop to force" << endl;
					state = FORCE;					
				}

			} else if (state == FORCE) {
				desired_task_pos = redis_client.getEigen(DESIRED_POSITION_KEY);

				pose_task->parametrizeForceMotionSpaces(1, Vector3d::UnitZ());
								
				pose_task->setClosedLoopForceControl();
				
				// goal force is picked depending on the force counter
				pose_task->setGoalForce(desired_forces(force_reading_counter)*Vector3d::UnitZ());
				// setting gains for force control
				pose_task->setForceControlGains(1.2, 3.0, 1.5);

				// setting desired position 
				pose_task->setGoalPosition(desired_task_pos);
				pose_task->setGoalOrientation(desired_orientation);

				// sets desired position of joints
				joint_task->setGoalPosition(q_desired);

				// update task model to set the null space
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// save current world force as previous world force every 250 loop iterations (0.25 seconds)
				if (loop_counter%250 == 0) {
					previous_world_sensed_z_forces[loop_counter/250-1] = world_sensed_force(2);
					// cout << "1" << endl;
					if (loop_counter/250 == previous_world_sensed_z_forces.size()){
						loop_counter = 1;
					}
				}
				loop_counter++;

				// calculate standard deviation of previous world forces
				double std_dev = sqrt((previous_world_sensed_z_forces.array() - previous_world_sensed_z_forces.mean()).square().sum() / (previous_world_sensed_z_forces.size() - 1));
				
				// printing out stuff for debugging
				// cout << std_dev << endl;
				// cout << loop_counter << endl;
				// cout << previous_world_sensed_z_forces << endl;
				// cout << force_reading_counter << endl;

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// increment force counter when force readings are above a threshold and difference between current and 
				// previous force readings are below a threshold
				if (world_sensed_force(2) > 0.5 & std_dev < 0.01) {
					// save force readings
					outputFile << "F" << force_reading_counter+1 << ": " << pose_task->getSensedForceControlWorldFrame()(2) << endl;
					outputFile << "D" << force_reading_counter+1 << ": " << robot->position(control_link, control_point)(2) << "\n" << endl;
					
					// to reset saved values				
					previous_world_sensed_z_forces[1] = 0;
					previous_world_sensed_z_forces[2] = 0;
					previous_world_sensed_z_forces[3] = 0;
					force_reading_counter++;
					if (force_reading_counter > desired_forces.size()-1) {
						// reset force reading counter back to 0
						force_reading_counter = 0;

						// set desired end-effector position back to initial position before force control was started
						if (location_counter == 0) {
							desired_task_pos = waypoint1;	
						} else if (location_counter == 1) {
							desired_task_pos = waypoint2;
						} else if (location_counter == 2) {
							desired_task_pos = waypoint3;
						}
						redis_client.setEigen(DESIRED_POSITION_KEY, desired_task_pos);

						// return z axis to motion space control
						pose_task->parametrizeForceMotionSpaces(0, Vector3d::UnitZ());
						
						cout << "FORCE to RESET" << endl;
						state = RESET;
					}
				}
			// checks if the controller is in the reset state, where the end effector moves back to the first waypoint after taking \
			several force readings
			} else if (state == RESET) {
				// sets desired position of the end effector
				pose_task->setGoalPosition(desired_task_pos);

				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// if case to check if end effector has reached back to the first waypoint
				if (pose_task->goalPositionReached(0.01)){
					
					// set desired end-effector position back to first waypoint
						if (location_counter == 0) {
							desired_task_pos << waypoint2;	
						} else if (location_counter == 1) {
							desired_task_pos << waypoint3;
						}
						redis_client.setEigen(DESIRED_POSITION_KEY, desired_task_pos);

					// increment location counter
					location_counter++;

					// reset time saved
					time_saved = false;

					state = MOTION;
					cout << "RESET to MOTION" << endl;
				}
			}
		}
		// sending position of end effector in the world frame to redis
		redis_client.setEigen(POSITION_KEY,robot->position(control_link, control_point));

		// sending orientation of ee to redis
		redis_client.setEigen(ORIENTATION_KEY,pose_task->getCurrentOrientation());

		// sending desired orientation of ee to redis
		redis_client.setEigen(DESIRED_ORIENTATION_KEY,pose_task->getGoalOrientation());

		// sending desired ee position in the world frame to redis
		redis_client.setEigen(ACTUAL_DESIRED_POSITION_KEY,pose_task->getGoalPosition());

		// sending desired joint angles to redis
		redis_client.setEigen(DESIRED_JOINT_ANGLES_KEY,joint_task->getGoalPosition());

		// saturate command torques if they exceed a certain value, currently set to 35 Nm
		VectorXd abs_command_torques = command_torques.cwiseAbs();
		if (abs_command_torques.maxCoeff() > 35) {
			// cout << abs_command_torques.maxCoeff() << endl;
			float scale = 35/abs_command_torques.maxCoeff();
			command_torques = command_torques*scale;
		}

        // send torques to redis to be obtained by sim or Kinova
        redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY,command_torques);

		// cout << pose_task->getGoalForce() << endl;
		// cout << pose_task->getGoalPosition() << endl;
		// cout << pose_task->getForceSpaceDimension() << endl;
		// cout << joint_task->getGoalPosition() << endl;
		// cout << state << endl;
		// cout << pose_task->getVelocitySaturationEnabled() << endl;
		// cout << location_counter << endl;
		// cout << runloop << endl;
    }

	// Close output file
    outputFile.close();

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
