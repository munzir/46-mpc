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
// Helper functions

/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick
void joystickEvents(char* b_, double* x, BalancingConfig& params, double jsFwdAmp_,
                    double jsSpinAmp_, bool& joystickControl_, KRANG_MODE& MODE_,
                    Vector6d& K_, double& js_forw, double& js_spin);

/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState);

/// Get the joint values from the encoders and the imu and compute the center of mass as well
void getState(Krang::Hardware* krang_, dart::dynamics::SkeletonPtr robot_, Vector6d& state, double dt, Eigen::Vector3d* com = NULL);
