/**
 * @file controller.cpp
 * @brief Controls forces in z at the end effector. Robot will hold a stationary position and switch to force control when a force
 * 			in the z axis is detected at the end effector.
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

// specify if simulation is on
bool simulation_flag = false;

// ------------------------------------------------------------------------------------------------------------------------

// States 
enum State {
	POSTURE = 0, 
	MOTION,
	FORCE,
	CONTACT
};

// initialize variable to store forces and moments in ee frame and world frame
Vector3d ee_sensed_force = Vector3d(0, 0, 0);
Vector3d world_sensed_force = Vector3d(0, 0, 0);
Vector3d ee_sensed_moment = Vector3d(0, 0, 0); 
Vector3d world_sensed_moment = Vector3d(0, 0, 0);

// initialize counter to keep track of force readings
int force_reading_counter = 0;
// initialize vector to keep track of previous world forces
Vector3d previous_world_sensed_z_forces = Vector3d(0, 0, 0);
// setting up variable to keep count of number of loop iterations
int loop_counter = 0;

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
	// if(!simulation_flag) {
		// const std::string JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot_command_torques::PANDA";
		// const std::string GRAVITY_COMP_ENABLED_KEY = "cs225a::simviz::gravity_comp_enabled";
		// const std::string Position = "Desired_Position";
		// const std::string POSITION_KEY = "cs225a::simviz::position";
		// const std::string ORIENTATION_KEY = "cs225a::simviz::orientation";
		// const std::string CONTROLLER_READY_KEY = "Controller_Ready";
	// }

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
	redis_client.setEigen(EE_FRAME_SENSED_MOMENT_KEY, ee_sensed_moment);
	redis_client.setEigen(WORLD_FRAME_SENSED_MOMENT_KEY, world_sensed_moment);

	// intialize vector for gravity
	Vector3d gravity = Vector3d(0,0,-9.81);
	// intialize rotation matrix for force sensor to ee frame
	Matrix3d force_sensor_rot_matrix;
	force_sensor_rot_matrix << 0, 1, 0,
								1, 0, 0,
								0, 0, -1;

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
	joint_task->setBoundedInertiaEstimateThreshold(0.15);
	joint_task->setGains(100, 20, 0);

	// turn on full dynamic decoupling if simulation flag is connected
	if (simulation_flag){
		pose_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
		joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	}

	// specify desired posture
	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 15, 0, 120, 0, 45, 67; // higher new home 2 position
	// q_desired.head(7) << 0, 25, 8, 106, 0, 47, 67; // new home 2 position
	// q_desired.head(7) << 0, 35, 8, 106, 0, 38, 67; // new home position
	// q_desired.head(7) << 0, 15, 180, -130, 0, 45, 90; // home position
	q_desired.head(7) *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

	// specify desired orientation
	MatrixXd desired_orientation = MatrixXd::Identity(3, 3);
	desired_orientation << 0,1,0,
						   -1,0,0,
						   0,0,1;

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
		if (!simulation_flag){
			// update force sensor values (needs to be the force applied by the robot to the environment, in sensor frame)
			ee_sensed_force = ee_sensed_force - pose_task->getCurrentOrientation().transpose()*0.174*gravity;
			// DEBUGGING->priting out calculated gravitational force from attached stethoscope mount
			cout << (pose_task->getCurrentOrientation()*0.174*gravity).transpose() << endl;
			pose_task->updateSensedForceAndMoment(-ee_sensed_force, -ee_sensed_moment);
		} else {
			pose_task->updateSensedForceAndMoment(-ee_sensed_force, -ee_sensed_moment);
		}

		// converts forces in ee frame to the world frame
		world_sensed_force = -pose_task->getSensedForceControlWorldFrame();
		world_sensed_moment = -pose_task->getSensedMomentControlWorldFrame();
		// send force in world frame to redis
		redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);
		redis_client.setEigen(WORLD_FRAME_SENSED_MOMENT_KEY, world_sensed_moment);

		if (controller_number = 1){
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

				// switch to force control if there is a force detected in the z-axis
				if (pose_task->getSensedForceControlWorldFrame()(2) <= -1.0){
					state = FORCE;
					cout << "motion to force" << endl;
				}
			}
			else if (state == FORCE) {

				desired_task_pos = redis_client.getEigen(DESIRED_POSITION_KEY);

				pose_task->parametrizeForceMotionSpaces(1, Vector3d::UnitZ());
								
				pose_task->setClosedLoopForceControl();

				pose_task->setGoalForce(-3*Vector3d::UnitZ());
				pose_task->setForceControlGains(0.7, 5.0, 0);
				
				// setting desired position and orientation
				pose_task->setGoalPosition(desired_task_pos);
				pose_task->setGoalOrientation(desired_orientation);

				// update task model to set the null space
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// save current world force as previous world force every 1500 loop iterations (1.5 seconds)
				if (loop_counter == 0) {
					previous_world_sensed_z_forces[0] = world_sensed_force(2);
					loop_counter++;
					// cout << "1" << endl;
				} else if (loop_counter == 1500) {
					previous_world_sensed_z_forces[1] = world_sensed_force(2);
					loop_counter++;
					// cout << "2" << endl;
				} else if (loop_counter == 3000) {
					previous_world_sensed_z_forces[2] = world_sensed_force(2);
					loop_counter = 0;
					// cout << "3" << endl;
				} else {
					loop_counter++;
				}

				// calculate standard deviation of previous world forces
				double std_dev = sqrt((previous_world_sensed_z_forces.array() - previous_world_sensed_z_forces.mean()).square().sum() / (previous_world_sensed_z_forces.size() - 1));

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				state = CONTACT;

			}
			else if (state == CONTACT){
				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				if (timer.elapsedCycles() % 1000 == 0) {
					cout << "Current force: " << pose_task->getSensedForceControlWorldFrame().transpose() << endl;
					cout << "Goal force: " << pose_task->getGoalForce().transpose() << endl;
					cout << endl;
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
