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

	// initialize stiffness values for plates, k1 for red plate, k2 for blue, k3 for green, values are obtained from redis
	float k1 = redis_client.getDouble("k1")/7;
	float k2 = redis_client.getDouble("k2")/7;
	float k3 = redis_client.getDouble("k3")/7;

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
	const Vector3d control_point = Vector3d(0, 0, -0.03);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	pose_task->setPosControlGains(100, 20, 0);
	pose_task->setOriControlGains(150, 15, 0);
	
	// initialize variables to store ee position and orientation
	Vector3d ee_pos;
	Matrix3d ee_ori;

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	joint_task->setGains(250, 15, 0);

	// specify desired posture
	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 20, 4, 130, 0, 30, 67;
	// q_desired.head(7) << 0, 25, 8, 106, 0, 47, 67; // new home 2 position
	// q_desired.head(7) << 0, 35, 8, 106, 0, 38, 67; // new home position
	// q_desired.head(7) << 0, 15, 180, -130, 0, 45, 90; // home position
	// q_desired.head(7) << 0, 0, 180, -90, 0, -90, 90; // tool facing down position
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

    // initialize desired task position, quaternion and rotation
    Vector3d desired_task_pos = Vector3d(0, 0, 0);
	VectorXd haply_quat(4);
	haply_quat << 0,0,0,0;
	MatrixXd desired_rotation = MatrixXd::Identity(3,3);

	// set ready signal for plotting to true
	redis_client.set("Ready_Signal", "True");
	
	// set desired end-effector position to zero
	redis_client.setEigen(DESIRED_POSITION_KEY, desired_task_pos);
	
	// print out obtained stiffness values
	cout << "k1: " << k1 << endl;
	cout << "k2: " << k2 << endl;
	cout << "k3: " << k3 << endl;

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

		// std::cout << robot->q().transpose() << "\n";

		if(controller_number = 1){
			// this if case makes the manipulator move to the desired joint positions
			if (state == POSTURE) {
				// update task model 
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);

				command_torques = joint_task->computeTorques() + robot->jointGravityVector();

				// once the manipulator achieves desired joint positions switches to task space control of the end effector
				if ((robot->q() - q_desired).norm() < 1e-2) {
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
				
				// sets desired position of the end effector
				pose_task->setGoalPosition(ee_pos + desired_task_pos);
				// sets desired orientation of the end effector
				pose_task->setGoalOrientation(desired_orientation);

				// save variable for world desired position
				Vector3d world_desired_position = ee_pos + desired_task_pos;

				// update task model
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				// have the joint task in the null space of the pose task
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
				
				// debugging->checking position error
				// cout << (robot->position(control_link, control_point)).transpose() << endl;
				// cout << (world_desired_position).transpose() << endl;
				
				// to compute the required torques to accomplish task
				command_torques = pose_task->computeTorques() + joint_task->computeTorques() + robot->jointGravityVector();

				// based on position of end effector change forces being sent to haply
				if (world_desired_position(2) < 0.16){
					if (world_desired_position(0) > 0.35 && world_desired_position(0) < 0.55 && world_desired_position(1) > -0.3 && world_desired_position(1) < -0.1){
						world_sensed_force(2) = k1*(0.16 - world_desired_position(2));
						cout << "touching plate 1" << endl;
						cout << world_sensed_force.transpose() << endl;
					}
					else if (world_desired_position(0) > 0.35 && world_desired_position(0) < 0.55 && world_desired_position(1) > -0.1 && world_desired_position(1) < 0.1){
						world_sensed_force(2) = k2*(0.16 - world_desired_position(2));
						cout << "touching plate 2" << endl;
						cout << world_sensed_force.transpose() << endl;
					}
					else if (world_desired_position(0) > 0.35 && world_desired_position(0) < 0.55 && world_desired_position(1) > 0.1 && world_desired_position(1) < 0.3){
						world_sensed_force(2) = k3*(0.16 - world_desired_position(2));
						cout << "touching plate 3" << endl;
						cout << world_sensed_force.transpose() << endl;
					}
				}
				else {
					world_sensed_force = Vector3d(0, 0, 0);
				}

				// sending forces to be rendered by haply
				redis_client.setEigen(WORLD_FRAME_SENSED_FORCE_KEY, world_sensed_force);
			}	 		
		}

		// sending position of end effector in the world frame to redis
		redis_client.setEigen(POSITION_KEY,robot->position(control_link, control_point));

		// cout << robot->position(control_link, control_point)(0) << endl;

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

	// send 0 torques to redis
    command_torques.setZero();
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY,command_torques);

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
