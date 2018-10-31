/**
 * @file helpers.h
 * @author Munzir
 * @date July 8th, 2013
 * @brief This file comtains some helper functions used for balancing
 */

#pragma once

#include <Eigen/Dense>

#include <dirent.h>
#include <iostream>

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <filter.h>
#include <imud.h>
#include <pciod.h>

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <initModules.h>

#include <kore.hpp>
#include "balancing_config.h"

/* ******************************************************************************************** */
// Constants for the robot kinematics
const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches

/* ******************************************************************************************** */
typedef Eigen::Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Eigen::Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Eigen::Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

/* ******************************************************************************************** */
// Globals for imu, motors and joystick

somatic_d_t daemon_cx;				///< The context of the current daemon

Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
dart::dynamics::SkeletonPtr robot;			///< the robot representation in dart

bool joystickControl = false;

double jsFwdAmp;				///< The gains for joystick forward/reverse input
double jsSpinAmp;				///< The gains for joystick left/right spin input

char b [10];						///< Stores the joystick button inputs
double x [6];


/* ******************************************************************************************** */
// Krang Mode Enum
enum KRANG_MODE {
    GROUND_LO,
    STAND,
    SIT,
    BAL_LO,
    BAL_HI,
    GROUND_HI,
    MPC
};

/* ******************************************************************************************** */
// All the freaking gains

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
// Constants for end-effector wrench estimation
//static const double eeMass = 2.3 + 0.169 + 0.000;			///< The mass of the robotiq end-effector
static const double eeMass = 1.6 + 0.169 + 0.000;			///< The mass of the Schunk end-effector
static const Eigen::Vector3d s2com (0.0, -0.008, 0.091); // 0.065 robotiq itself, 0.026 length of ext + 2nd

/* ******************************************************************************************** */
// Helper functions

/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick
void joystickEvents(char* b_, double* x, BalancingConfig& params, double jsFwdAmp_,
                    double jsSpinAmp_, bool& joystickControl_, KRANG_MODE& MODE_,
                    Vector6d& K_, double& js_forw, double& js_spin);

/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState);

/// Get the joint values from the encoders and the imu and compute the center of mass as well
void getState(Krang::Hardware* krang_, dart::dynamics::SkeletonPtr robot_, Vector6d& state, double dt, Eigen::Vector3d* com = NULL);
