/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot with 1 DOF gripper 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd ui_torques;
mutex mutex_torques, mutex_update;

// specify urdf and robots 
const string world_file = "./resources/world_measure_heartbeat.urdf";
const string robot_file = "./resources/GEN3_URDF_V12_stethoscope.urdf";
// const string robot_file = "./resources/GEN3_URDF_V12.urdf";
const string robot_name = "kinova";
const string camera_name = "camera_fixed";

// dynamic objects information
const vector<std::string> object_names = {"cup"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

// global variables for setting up simulated force sensor
// const string link_name = "end-effector";
// const Vector3d sensor_pos_in_link = Vector3d(0, 0, -0.03);
const string link_name = "stethoscope";
const Vector3d sensor_pos_in_link = Vector3d(0, 0, -0.0377);


// global variables for sensed force and moment
Vector3d ee_sensed_force;
Vector3d ee_sensed_moment;


int main() {
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
	graphics->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	// graphics->showLinkFrame(true, robot_name, "link7", 0.15);  // can add frames for different links
	// graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 
	graphics->addUIForceInteraction(robot_name);
	// graphics->showWireMesh(true,robot_name);

	// load robots
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);

	// Change starting position
	int dof = robot->dof();
	cout << dof << endl;
	VectorXd q_desired(dof);
	q_desired.head(dof) << -20, 15, 0, 120, 0, 45, 67; // new home 2 position
	// q_desired.head(dof) << 0, 0, 0, 90, 0, 90, 67, 0, 0; // with gripper position
	// q_desired.head(dof) << 0, 15, 180, -130, 0, 55, 90;
	q_desired.head(dof) *= M_PI / 180.0;
	robot->setQ(q_desired);
	// robot->setDq();
	robot->updateModel();
	ui_torques = VectorXd::Zero(robot->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());
	// cout << sim->getWorldGravity() << endl;

	// create simulated force sensor
	Affine3d T_sensor = Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
	sim->addSimulatedForceSensor(robot_name, link_name, T_sensor, 15.0);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);

	// // fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q()); 
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq()); 
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());

	// start simulation thread
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
        graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));

		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

    // stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// fSimulationRunning = true;

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

	// create a timer
	double sim_freq = 2000;
	Sai2Common::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);
	sim->enableJointLimits(robot_name);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		double time = timer.elapsedSimTime();
		VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);

		// force and moment sensor update
		ee_sensed_force = sim->getSensedForce(robot_name, link_name);
		ee_sensed_moment = sim->getSensedMoment(robot_name, link_name);
		ee_sensed_force = -ee_sensed_force; // sensed force should be opposite sign
		ee_sensed_moment = -ee_sensed_moment; // sensed force should be opposite sign

		if (control_torques.norm() > 0){
			sim->enableGravityCompensation(false);
		}
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}
		sim->integrate();
        redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
        redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));

		// update object information 
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				object_poses[i] = sim->getObjectPose(object_names[i]);
				object_velocities[i] = sim->getObjectVelocity(object_names[i]);
			}
		}

		// send force and moment in ee frame to redis
		redis_client.setEigen(EE_FRAME_SENSED_FORCE_KEY, ee_sensed_force);
		redis_client.setEigen(EE_FRAME_SENSED_MOMENT_KEY, ee_sensed_moment);
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}