/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and -+++++++++ing...
 */

#include "helpers.h"
#include "balancing_config.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <kore.hpp>
#include <math.h>

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well
void getState(Krang::Hardware* krang_, dart::dynamics::SkeletonPtr robot_, Vector6d& state, double dt, Eigen::Vector3d* com_) {

  // Read motor encoders, imu and ft and update dart skeleton
  krang_->updateSensors(dt);

  // Calculate the COM Using Skeleton
  Eigen::Vector3d com = robot_->getCOM() - robot_->getPositions().segment(3,3);
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
void joystickEvents(char* b_, double* x, BalancingConfig& params, double jsFwdAmp_,
                    double jsSpinAmp_, bool& joystickControl_, KRANG_MODE& MODE_,
                    Vector6d& K_, double& js_forw, double& js_spin) {

  // Change the gains with the given joystick input
  double deltaTH = 0.2, deltaX = 0.02, deltaSpin = 0.02;
  if(!joystickControl_) {
    for(size_t i = 0; i < 4; i++) {
      if(((b_[5] == 0) && (b_[7] == 0)) && (b_[i] == 1)) K_(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
      else if((b_[5] == 1) && (b_[i] == 1)) K_((i % 2) + 2) += ((i < 2) ? deltaX : -deltaX);
      else if((b_[7] == 1) && (b_[i] == 1)) K_((i % 2) + 4) += ((i < 2) ? deltaSpin : -deltaSpin);
    }
  }

  // Update joystick and force-compensation controls
  static int lastb0 = b_[0], lastb1 = b_[1], lastb2 = b_[2];

  if((b_[4] == 1) && (b_[6] == 0) && (b_[0] == 1) && (lastb0 == 0)) {
    joystickControl_ = !joystickControl_;
    if(joystickControl_ == true) {
      somatic_motor_reset(&daemon_cx, krang->arms[Krang::LEFT]);
      somatic_motor_reset(&daemon_cx, krang->arms[Krang::RIGHT]);
    }
  }

  if((b_[4] == 1) && (b_[6] == 0) && (b_[2] == 1) && (lastb2 == 0)) {
    if(MODE_ == BAL_LO) {
      printf("Mode 5\n");
      K_ = params.pdGainsBalHi;
      MODE_ = BAL_HI;
    }
    else if (MODE_ == BAL_HI) {
      printf("Mode 4\n");
      K_ = params.pdGainsBalLo;
      MODE_ = BAL_LO;
    }
  }

  lastb0 = b_[0], lastb1 = b_[1], lastb2 = b_[2];

  // Ignore the joystick statements for the arm control
  if((b_[4] == 1) || (b_[5] == 1) || (b_[6] == 1) || (b_[7] == 1)) {
    js_forw = js_spin = 0.0;
    return;
  }

  // Set the values for the axis
  if(MODE_ == GROUND_LO || MODE_ == GROUND_HI) {
    js_forw = -params.joystickGainsGroundLo(0) * x[1];
    js_spin = params.joystickGainsGroundLo(1) * x[2];
  }
  else if(MODE_ == BAL_LO) {
    js_forw = -params.joystickGainsBalLo(0) * x[1];
    js_spin = params.joystickGainsBalLo(1) * x[2];
  }
  else if(MODE_ == BAL_HI) {
    js_forw = -params.joystickGainsBalHi(0) * x[1];
    js_spin = params.joystickGainsBalHi(1) * x[2];
  }
  else {
    js_forw = -x[1] * jsFwdAmp_;
    js_spin = x[2] * jsSpinAmp_;
  }
}
