/**
 * @file force_moment_controller.cpp
 * @brief Has the robot do force and zero moment control of the end effector.
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

// specify whether to go to approach mode
bool go_to_approach_flag = true;

// specify whether to hold end effector position when doing moment control
bool hold_position_flag = true;

// specify if simulation is activated
bool simulation_flag = false;

// specify desired orientation
// flat orientation
// MatrixXd desired_orientation {{0,1,0},
// 					   		{-1,0,0},
// 					   		{0,0,1}};
// tilt along world y axis
// MatrixXd desired_orientation {{0,cos(M_PI/9),sin(M_PI/9)},
// 							{-1,0,0},
// 							{0,-sin(M_PI/9),cos(M_PI/9)}};
// tilt along world x axis
// MatrixXd desired_orientation {{0,1,0}, 
// 					   		{-cos(-M_PI/9),0,-sin(-M_PI/9)},
// 					   		{-sin(-M_PI/9),0,cos(-M_PI/9)}};
// pointed along x axis
MatrixXd desired_orientation {{0,0,-1},
							{-1,0,0},
							{0,1,0}};

// setting up desired positions 
Vector3d waypoint1 = Vector3d(0.60, 0, 0.24);

// ------------------------------------------------------------------------------------------------------------------------

// States 
enum State {
	POSTURE = 0, 
	MOTION,
	FORCE,
	APPROACH,
	CONTACT,
	RESET
};

// initialize variable to store forces and moments in ee frame and world frame
Vector3d ee_sensed_force = Vector3d(0, 0, 0);
Vector3d world_sensed_force = Vector3d(0, 0, 0);
Vector3d ee_sensed_moment = Vector3d(0, 0, 0);
Vector3d world_sensed_moment = Vector3d(0, 0, 0);

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

	// config file names and object names
	string robot_file;
	if (!stethoscope) {
		robot_file = "./resources/GEN3_URDF_V12.urdf";
	} else {
		robot_file = "./resources/GEN3_URDF_V12_stethoscope.urdf";
 	}

	// TESTING --> set up different set of keys if we are connected to actual kinova robot
	if(!simulation_flag) {
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

	// intialize vector for gravity
	Vector3d gravity = Vector3d(0,0,-9.81);

	// initialize vector for position of center of mass of stethoscope
	Vector3d stethoscope_cg_pos = Vector3d(0,0,-0.03);

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
	pose_task->setBoundedInertiaEstimateThreshold(0.20);
	pose_task->setPosControlGains(100, 20, 0);
	pose_task->setOriControlGains(100, 20, 0);
	// set the force sensor location for the contact part of the task
	pose_task->setForceSensorFrame(control_link, Affine3d::Identity());

	// initialize time
	double initial_time = 0;
	bool time_saved = false;

	// initialize flag to keep track whether time has been saved in contact state
	bool contact_time_saved_flag = false;
	// variable to keep track of time contact state starts
	double contact_time;

	// initialize desired task position
	Vector3d desired_task_pos = Vector3d(0, 0, 0);

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->setBoundedInertiaEstimateThreshold(0.15);
	joint_task->setGains(100, 20, 0);

	// turn on full dynamic decoupling if simulation flag is activated
	if (simulation_flag){
		pose_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
		joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	}

	// specify desired posture
	VectorXd q_desired(dof);
	// q_desired.head(7) << 0, 20, 0, 130, 0, 30, 67; // new home 2 position
	q_desired.head(7) << 0, 15, 0, 120, 0, 45, 67; // higher new home 2 position
	// q_desired.head(7) << 0, 15, 180, -130, 0, 45, 90; // home position
	q_desired.head(7) *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

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

		// compensate for weight of mount when connected to hardware
		if (!simulation_flag && stethoscope){
			// update force sensor values (needs to be the force applied by the robot to the environment, in sensor frame)
			ee_sensed_moment = ee_sensed_moment - stethoscope_cg_pos.cross(pose_task->getCurrentOrientation().transpose()*0.174*gravity);
			ee_sensed_force = ee_sensed_force - pose_task->getCurrentOrientation().transpose()*0.174*gravity;
			// DEBUGGING->priting out calculated gravitational force from attached stethoscope mount
			// cout << (pose_task->getCurrentOrientation()*0.174*gravity).transpose() << endl;
			pose_task->updateSensedForceAndMoment(-ee_sensed_force, -ee_sensed_moment);
		} else {
			pose_task->updateSensedForceAndMoment(-ee_sensed_force, -ee_sensed_moment);
		}

		// get sensed forces and moments in world frame
		world_sensed_force = -pose_task->getSensedForceControlWorldFrame();
		world_sensed_moment = -pose_task->getSensedMomentControlWorldFrame();
		// send forces and moments in world frame to redis
		redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);
		redis_client.setEigen(WORLD_FRAME_SENSED_MOMENT_KEY, world_sensed_moment);

		// DEBUGGING -> checking mass matrix of kinova
		// cout << robot->M() << '\n' << endl;

		if(controller_number = 1){
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
					
					// enable velocity saturation of operational point default is 0.3 and M_PI/3
					pose_task->enableVelocitySaturation(linear_velocity_sat, angular_velocity_sat);

					state = MOTION;
				}
			// this if case controls the end effector position in task space
			} else if (state == MOTION) {
				// obtain desired postion from redis which is usually updated by Haply inverse3
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

				// switch to approach state if we reach desired position and orientation
				if (pose_task->goalPositionReached(0.015) && pose_task->goalOrientationReached(0.035) && go_to_approach_flag && !contact_time_saved_flag){
					// save time we reach the goal position
					if (!time_saved){
						initial_time = time;
						time_saved = true;						
					}
					
					// after 4 seconds of reaching goal position, switch to approach state
					if (time - initial_time > 4){

						// disable velocity saturation of operational point
						pose_task->disableVelocitySaturation();

						// TESTING->lowering gains when going into approach state
						pose_task->setPosControlGains(50, 14, 0);

						cout << "motion to approach" << endl;
						state = APPROACH;

					}
				}
			} else if (state == APPROACH){
				
				// manually changing desired position to move in the positive x direction at 0.007 m/s
				desired_task_pos[0] += 0.000007;
				redis_client.setEigen(DESIRED_POSITION_KEY,desired_task_pos);
				pose_task->setGoalPosition(desired_task_pos);

				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// swap to force state if a force is detected at ee in the x axis
				if (pose_task->getSensedForceControlWorldFrame()(0) >= 1.0){
					
					cout << "approach to force" << endl;
					state = FORCE;					

				}

			} else if (state == FORCE) {
				desired_task_pos = redis_client.getEigen(DESIRED_POSITION_KEY);

				pose_task->parametrizeMomentRotMotionSpaces(2, Vector3d::UnitX());
				// set all 3 axes to be controlled in force if hold position is set to false
				if (hold_position_flag) {
					pose_task->parametrizeForceMotionSpaces(1, Vector3d::UnitX());
				} else {
					pose_task->parametrizeForceMotionSpaces(3);	
				}				
				
				pose_task->setClosedLoopForceControl();
				pose_task->setClosedLoopMomentControl();

				pose_task->setGoalMoment(Vector3d::Zero()); // set desired moments to zero
				pose_task->setGoalForce(4*Vector3d::UnitX()); // set desired force to 4 N

				pose_task->setForceControlGains(0.7, 5.0, 0);
				// pose_task->setMomentControlGains(0.7, 4.0, 0.0); // default moment control gains
				pose_task->setMomentControlGains(14.0, 5.0, 0);

				pose_task->setGoalPosition(desired_task_pos);
				// pose_task->setGoalOrientation(desired_orientation);

				// TESTING->lowering gains on positional and orientation control of operational point
				pose_task->setPosControlGains(50, 14, 0);
				pose_task->setOriControlGains(25, 10, 0);

				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// set contact time saved tp true and save time
				contact_time_saved_flag = true;
				contact_time = time;

				// swap to contact state
				state = CONTACT;

			} else if (state == CONTACT){
				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// DEBUGGING->printing out commanded torques for each task
				cout << "pose_task torq" << endl;
				cout << pose_task->computeTorques().transpose() << endl;
				cout << "joint_task torq" << endl;
				cout << joint_task->computeTorques().transpose() << endl;
				cout << "gravity torq" << endl;
				cout << robot->jointGravityVector().transpose() << endl;

				// DEBUGGING->printing out sensed and desired forces and moments
				if (timer.elapsedCycles() % 1000 == 0) {
					cout << "Current force: "
						<< pose_task->getSensedForceControlWorldFrame().transpose() << endl;
					cout << "Current moment: "
						<< pose_task->getSensedMomentControlWorldFrame().transpose() << endl;
					cout << "Goal force: "
						<< pose_task->getGoalForce().transpose() << endl;
					cout << "Goal moment: "
						<< pose_task->getGoalMoment().transpose() << endl;
					cout << endl;
				}

				// swap back to to motion state after a certain amount of time in the contact state
				if (time - contact_time > 50) {
					
					// turn off force control
					pose_task->parametrizeForceMotionSpaces(0);
					pose_task->parametrizeMomentRotMotionSpaces(0);

					// turn velocity saturation back on
					pose_task->enableVelocitySaturation(linear_velocity_sat, angular_velocity_sat);

					// set original gains for both position and orientation
					pose_task->setPosControlGains(100, 20, 0);
					pose_task->setOriControlGains(100, 20, 0);

					// set desired location back to waypoint 1
					desired_task_pos = waypoint1;
					redis_client.setEigen(DESIRED_POSITION_KEY, desired_task_pos);

					// swap to motion state
					state = MOTION;

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

		// DEBUGGING->printing out mass matrix (non saturated) for arm
		// cout << robot->M() << endl;
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
