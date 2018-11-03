/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file control.cpp
 * @author Munzir Zafar
 * @date Oct 31, 2018
 * @brief Implements balancing control functions
 */

#include "control.h"

#include <algorithm>
#include <iostream>

#include <dart/dart.hpp>
#include <Eigen/Eigen>
#include <kore.hpp>
#include <somatic.h>

#include "balancing_config.h"

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well
void getState(Krang::Hardware* krang_, dart::dynamics::SkeletonPtr robot_,
              Eigen::Matrix<double, 6, 1>& state, double dt, Eigen::Vector3d* com_) {

  // Read motor encoders, imu and ft and update dart skeleton
  krang_->updateSensors(dt);

  // Calculate the COM Using Skeleton
  Eigen::Vector3d com = robot_->getCOM() - robot_->getPositions().segment(3,3);
  if(com_ != NULL) *com_ = com;

  // Update the state (note for amc we are reversing the effect of the motion of the upper body)
  // State are theta, dtheta, x, dx, psi, dpsi
  state(0) = atan2(com(0), com(2)); // - 0.3 * M_PI / 180.0;;
  state(1) = krang_->imuSpeed;
  state(2) = (krang_->amc->pos[0] + krang_->amc->pos[1])/2.0 + krang_->imu;
  state(3) = (krang_->amc->vel[0] + krang_->amc->vel[1])/2.0 + krang_->imuSpeed;
  state(4) = (krang_->amc->pos[1] - krang_->amc->pos[0]) / 2.0;
  state(5) = (krang_->amc->vel[1] - krang_->amc->vel[0]) / 2.0;

  // Making adjustment in com to make it consistent with the hack above for state(0)
  com(0) = com(2) * tan(state(0));
}

/* ******************************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (const BalancingConfig& params, const KRANG_MODE& MODE_,
                      const double& js_forw, const double& js_spin,
                      const double& jsFwdAmp_, const double& jsSpinAmp_,
                      const double& dt, Eigen::Matrix<double, 6, 1>& refState) {

  double forw, spin;
  // Set the values for the axis
  if(MODE_ == GROUND_LO || MODE_ == GROUND_HI) {
    forw = params.joystickGainsGroundLo(0) * js_forw;
    spin = params.joystickGainsGroundLo(1) * js_spin;
  }
  else if(MODE_ == BAL_LO) {
    forw = params.joystickGainsBalLo(0) * js_forw;
    spin = params.joystickGainsBalLo(1) * js_spin;
  }
  else if(MODE_ == BAL_HI) {
    forw = params.joystickGainsBalHi(0) * js_forw;
    spin = params.joystickGainsBalHi(1) * js_spin;
  }
  else {
    forw = js_forw * jsFwdAmp_;
    spin = js_spin * jsSpinAmp_;
  }

  // First, set the balancing angle and velocity to zeroes
  refState(0) = refState(1) = 0.0;

  // Set the distance and heading velocities using the joystick input
  refState(3) = forw;
  refState(5) = spin;

  // Integrate the reference positions with the current reference velocities
  refState(2) += dt * refState(3);
  refState(4) += dt * refState(5);
}

/* ********************************************************************************************* */
/// Handles the wheel commands if we are started
void BalanceControl(somatic_d_t& daemon_cx_, bool start_, bool joystickControl_,
                    KRANG_MODE MODE_,  Eigen::Matrix<double, 6, 1>& K_,
                    Eigen::Matrix<double, 6, 1>& error, bool debug,
                    double * control_input) {

  // Compute the current
  double u_theta = K_.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());
  double u_x = K_(2)*error(2) + K_(3)*error(3);
  double u_spin =  -K_.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
  u_spin = std::max(-30.0, std::min(30.0, u_spin));

  // Compute the input for left and right wheels
  if(joystickControl_ && ((MODE_ == GROUND_LO) || (MODE_ == GROUND_HI))) {u_x = 0.0; u_spin = 0.0;}
  control_input[0] = std::max(-49.0, std::min(49.0, u_theta + u_x + u_spin));
  control_input[1] = std::max(-49.0, std::min(49.0, u_theta + u_x - u_spin));
  if(debug) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
}

/* ******************************************************************************************** */
// // Change robot's beta values (parameters)
dart::dynamics::SkeletonPtr setParameters(dart::dynamics::SkeletonPtr robot_,
                                          Eigen::MatrixXd betaParams, int bodyParams) {
  Eigen::Vector3d bodyMCOM;
  double mi;
  int numBodies = betaParams.cols()/bodyParams;
  for (int i = 0; i < numBodies; i++) {
    mi = betaParams(0, i * bodyParams);
    bodyMCOM(0) = betaParams(0, i * bodyParams + 1);
    bodyMCOM(1) = betaParams(0, i * bodyParams + 2);
    bodyMCOM(2) = betaParams(0, i * bodyParams + 3);

    robot_->getBodyNode(i)->setMass(mi);
    robot_->getBodyNode(i)->setLocalCOM(bodyMCOM/mi);
  }
  return robot_;
}

/* ******************************************************************************************** */
void BalancingController(const Krang::Hardware* krang,
                         const Eigen::Matrix<double, 6, 1>& state,
                         const double& dt,
                         const BalancingConfig& params,
                         const bool joystickControl,
                         const double& js_forw, const double& js_spin,
                         const bool& debug,
                         KRANG_MODE& MODE,
                         Eigen::Matrix<double, 6, 1>& refState,
                         Eigen::Matrix<double, 6, 1>& error,
                         double* control_input) {

  Eigen::Matrix<double, 6, 1> K;

  switch (MODE) {
    case GROUND_LO: {
      K = params.pdGainsGroundLo;

      // Update Reference
      double forw, spin;
      // Set the values for the axis
      if(MODE == GROUND_LO || MODE == GROUND_HI) {
        forw = params.joystickGainsGroundLo(0) * js_forw;
        spin = params.joystickGainsGroundLo(1) * js_spin;
      }
      else if(MODE == BAL_LO) {
        forw = params.joystickGainsBalLo(0) * js_forw;
        spin = params.joystickGainsBalLo(1) * js_spin;
      }
      else if(MODE == BAL_HI) {
        forw = params.joystickGainsBalHi(0) * js_forw;
        spin = params.joystickGainsBalHi(1) * js_spin;
      }
      else {
        forw = 0.0;
        spin = 0.0;
      }

      // First, set the balancing angle and velocity to zeroes
      refState(0) = refState(1) = 0.0;

      // Set the distance and heading velocities using the joystick input
      refState(3) = forw;
      refState(5) = spin;

      // Integrate the reference positions with the current reference velocities
      refState(2) += dt * refState(3);
      refState(4) += dt * refState(5);

      if(debug) std::cout << "refState: " << refState.transpose() << std::endl;

      // Calculate state Error
      error = state - refState;
      if(debug) {
        std::cout << "error: " << error.transpose();
        std::cout << ", imu: " << krang->imu / M_PI * 180.0 << std::endl;
      }

      static int mode4iter = 0;
      size_t mode4iterLimit = 100;
      // If in ground Lo mode and waist angle increases beyond 150.0 goto groundHi mode
      if(MODE == GROUND_LO) {
        if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 < 150.0*M_PI/180.0) {
          MODE = GROUND_HI;
          K = params.pdGainsGroundHi;
        }
      }
      // If in ground Hi mode and waist angle decreases below 150.0 goto groundLo mode
      else if(MODE == GROUND_HI) {
        if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
          MODE = GROUND_LO;
          K = params.pdGainsGroundLo;
        }
      }

      // If we are in the sit down mode, over write the reference
      else if(MODE == SIT) {
        static const double limit = ((-103.0 / 180.0) * M_PI);
        if(krang->imu < limit) {
          printf("imu (%lf) < limit (%lf): changing to mode 1\n", krang->imu, limit);
          MODE = GROUND_LO;
          K = params.pdGainsGroundLo;
        }
        else error(0) = krang->imu - limit;
      }
      // if in standing up mode check if the balancing angle is reached and stayed,
      // if so switch to balLow mode
      else if(MODE == STAND) {
        if(fabs(state(0)) < 0.034) mode4iter++;
        // Change to mode 4 (balance low) if stood up enough
        if(mode4iter > mode4iterLimit) {
          MODE = BAL_LO;
          mode4iter = 0;
          K = params.pdGainsBalLo;
        }
      }

      break;
    }
    case GROUND_HI: {
      break;
    }
    case STAND: {
      break;
    }
    case BAL_LO: {
      break;
    }
    case BAL_HI: {
      break;
    }
    case SIT: {
      break;
    }
  }
}
