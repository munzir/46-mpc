/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and -+++++++++ing...
 */

#include "helpers.h"

#include <iostream>
#include <fstream>
#include <sstream>

/* ******************************************************************************************** */
// Initialize the gains for controller and joystick
KRANG_MODE MODE = GROUND_LO;
Vector6d K_groundLo;
Vector6d K_groundHi;
Eigen::Vector2d J_ground (1.0, 1.0);
Vector6d K_stand;
Eigen::Vector2d J_stand;
Vector6d K_sit;
Eigen::Vector2d J_sit;
Vector6d K_balLow;
Eigen::Vector2d J_balLow;
Vector6d K_balHigh;
Eigen::Vector2d J_balHigh;
Vector6d K = K_groundLo;

/* ******************************************************************************************** */
// Constants for the robot kinematics
const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches

/* ******************************************************************************************** */
// Initialize global variables

somatic_d_t daemon_cx;				///< The context of the current daemon
bool start = false;						///< Giving time to the user to get the robot in balancing angle
bool joystickControl = false;


Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
dart::simulation::WorldPtr world;			///< the world representation in dart
dart::dynamics::SkeletonPtr robot;			///< the robot representation in dart

double jsFwdAmp;				///< The gains for joystick forward/reverse input
double jsSpinAmp;				///< The gains for joystick left/right spin input

char b [10];						///< Stores the joystick button inputs
double x [6];

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well
void getState(Vector6d& state, double dt, Eigen::Vector3d* com_) {

	// Read motor encoders, imu and ft and update dart skeleton
	krang->updateSensors(dt);

	// Calculate the COM Using Skeleton
  	Eigen::Vector3d com = robot->getCOM() - robot->getPositions().segment(3,3);
  	if(com_ != NULL) *com_ = com;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	// State are theta, dtheta, x, dx, psi, dpsi
	state(0) = atan2(com(0), com(2)); // - 0.3 * M_PI / 180.0;;
	state(1) = krang->imuSpeed;
	state(2) = (krang->amc->pos[0] + krang->amc->pos[1])/2.0 + krang->imu;
	state(3) = (krang->amc->vel[0] + krang->amc->vel[1])/2.0 + krang->imuSpeed;
	state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;

	// Making adjustment in com to make it consistent with the hack above for state(0)
	com(0) = com(2) * tan(state(0));
}

/* ******************************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState) {

	// First, set the balancing angle and velocity to zeroes
	refState(0) = refState(1) = 0.0;

	// Set the distance and heading velocities using the joystick input
	refState(3) = js_forw;
	refState(5) = js_spin;

	// Integrate the reference positions with the current reference velocities
	refState(2) += dt * refState(3);
	refState(4) += dt * refState(5);
}

/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick
void joystickEvents(double& js_forw, double& js_spin) {

	// Change the gains with the given joystick input
	double deltaTH = 0.2, deltaX = 0.02, deltaSpin = 0.02;
	if(!joystickControl) {
		for(size_t i = 0; i < 4; i++) {
			if(((b[5] == 0) && (b[7] == 0)) && (b[i] == 1)) K(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
			else if((b[5] == 1) && (b[i] == 1)) K((i % 2) + 2) += ((i < 2) ? deltaX : -deltaX);
			else if((b[7] == 1) && (b[i] == 1)) K((i % 2) + 4) += ((i < 2) ? deltaSpin : -deltaSpin);
		}
	}

	// Update joystick and force-compensation controls
	static int lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];

	if((b[4] == 1) && (b[6] == 0) && (b[0] == 1) && (lastb0 == 0)) {
		joystickControl = !joystickControl;
		if(joystickControl == true) {
			somatic_motor_reset(&daemon_cx, krang->arms[Krang::LEFT]);
			somatic_motor_reset(&daemon_cx, krang->arms[Krang::RIGHT]);
		}
	}

	if((b[4] == 1) && (b[6] == 0) && (b[2] == 1) && (lastb2 == 0)) {
		if(MODE == BAL_LO) {
			printf("Mode 5\n");
			K = K_balHigh;
			MODE = BAL_HI;
		}
		else if (MODE == BAL_HI) {
			printf("Mode 4\n");
			K = K_balLow;
			MODE = BAL_LO;
		}
	}

	lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];

	// Ignore the joystick statements for the arm control
	if((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1)) {
		js_forw = js_spin = 0.0;
		return;
	}

	// Set the values for the axis
	if(MODE == GROUND_LO || MODE == GROUND_HI) {
		js_forw = -J_ground(0) * x[1], js_spin = J_ground(1) * x[2];
	}
	else if(MODE == BAL_LO) {
		js_forw = -J_balLow(0) * x[1], js_spin = J_balLow(1) * x[2];
	}
	else if(MODE == BAL_HI) {
		js_forw = -J_balHigh(0) * x[1], js_spin = J_balHigh(1) * x[2];
	}
	else {
		js_forw = -x[1] * jsFwdAmp;
		js_spin = x[2] * jsSpinAmp;;
	}
}
