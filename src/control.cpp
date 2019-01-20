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

#include <algorithm>  // std::max(), std::min()
#include <cmath>      // atan2, tan
#include <iostream>   // std::cout, std::endl
#include <string>     // std::string

#include <amino/time.h>  // aa_tm: _now(), _timespec2sec(), _sub()
#include <Eigen/Eigen>  // Eigen:: MatrixXd, VectorXd, Vector3d, Matrix<double, #, #>
#include <dart/dart.hpp>  // dart::dynamics::SkeletonPtr
#include <kore.hpp>       // Krang::Hardware

#include "adrc.hpp"            // computeLinearizedDynamics()
#include "balancing_config.h"  // BalancingConfig
#include "file_ops.hpp"        // readInputFileAsMatrix()
#include "lqr.hpp"             // lqr()

const char BalanceControl::MODE_STRINGS[][16] = {
    "Ground Lo", "Stand", "Sit", "Bal Lo", "Bal Hi", "Ground Hi"};
BalanceControl::BalanceControl(Krang::Hardware* krang_,
                               dart::dynamics::SkeletonPtr robot_,
                               BalancingConfig& params)
    : krang(krang_), robot(robot_) {
  // LQR Gains
  dynamicLQR = params.dynamicLQR;
  lqrQ = params.lqrQ;
  lqrR = params.lqrR;

  // PD Gains for all modes
  pdGains[BalanceControl::GROUND_LO] = params.pdGainsGroundLo;
  pdGains[BalanceControl::GROUND_HI] = params.pdGainsGroundHi;
  pdGains[BalanceControl::STAND] = params.pdGainsStand;
  pdGains[BalanceControl::SIT] = params.pdGainsSit;
  pdGains[BalanceControl::BAL_LO] = params.pdGainsBalLo;
  pdGains[BalanceControl::BAL_HI] = params.pdGainsBalHi;

  // Joystick Gains for all modes
  for (int i = 0; i < 2; i++) {
    joystickGains[BalanceControl::GROUND_LO][i] =
        params.joystickGainsGroundLo[i];
    joystickGains[BalanceControl::GROUND_HI][i] =
        params.joystickGainsGroundHi[i];
    joystickGains[BalanceControl::STAND][i] = params.joystickGainsStand[i];
    joystickGains[BalanceControl::SIT][i] = params.joystickGainsSit[i];
    joystickGains[BalanceControl::BAL_LO][i] = params.joystickGainsBalLo[i];
    joystickGains[BalanceControl::BAL_HI][i] = params.joystickGainsBalHi[i];
  }

  // Parameters used for generating internal events
  toBalThreshold = params.toBalThreshold;
  imuSitAngle = params.imuSitAngle;

  // Initial values
  balance_mode_ = BalanceControl::GROUND_LO;
  K = pdGains[BalanceControl::GROUND_LO];
  refState.setZero();
  state.setZero();
  error.setZero();
  joystick_forw = 0.0;
  joystick_spin = 0.0;

  // lqrHackRatios
  Eigen::VectorXd lqrGains = Eigen::VectorXd::Zero(4);
  lqrHackRatios = Eigen::Matrix<double, 4, 4>::Identity();
  lqrGains = BalanceControl::ComputeLqrGains();
  for (int i = 0; i < lqrHackRatios.cols(); i++) {
    lqrHackRatios(i, i) = pdGains[BalanceControl::STAND](i) / -lqrGains(i);
  }

  // Read CoM estimation model paramters
  Eigen::MatrixXd beta;
  std::string inputBetaFilename = params.comParametersPath;
  try {
    std::cout << "Reading converged beta ...\n";
    beta = readInputFileAsMatrix(inputBetaFilename);
    std::cout << "|-> Done\n";
  } catch (exception& e) {
    std::cout << e.what() << std::endl;
    assert(false && "Problem loading CoM parameters...");
  }
  BalanceControl::SetComParameters(beta, 4);

  // time
  t_prev = aa_tm_now();
}

double BalanceControl::ElapsedTimeSinceLastCall() {
  t_now = aa_tm_now();
  dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));
  t_prev = t_now;

  return dt;
}
void BalanceControl::UpdateState() {
  // Read motor encoders, imu and ft and update dart skeleton
  krang->updateSensors(dt);

  // Calculate the COM Using Skeleton
  com = robot->getCOM() - robot->getPositions().segment(3, 3);

  // Update the state (note for amc we are reversing the effect of the motion of
  // the upper body) State are theta, dtheta, x, dx, psi, dpsi
  state(0) = atan2(com(0), com(2));  // - 0.3 * M_PI / 180.0;;
  state(1) = krang->imuSpeed;
  state(2) = (krang->amc->pos[0] + krang->amc->pos[1]) / 2.0 + krang->imu;
  state(3) = (krang->amc->vel[0] + krang->amc->vel[1]) / 2.0 + krang->imuSpeed;
  state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
  state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;

  // Making adjustment in com to make it consistent with the hack above for
  // state(0)
  com(0) = com(2) * tan(state(0));
}

void BalanceControl::SetComParameters(Eigen::MatrixXd beta_params,
                                      int num_body_params) {
  Eigen::Vector3d bodyMCOM;
  double mi;
  int numBodies = beta_params.cols() / num_body_params;
  for (int i = 0; i < numBodies; i++) {
    mi = beta_params(0, i * num_body_params);
    bodyMCOM(0) = beta_params(0, i * num_body_params + 1);
    bodyMCOM(1) = beta_params(0, i * num_body_params + 2);
    bodyMCOM(2) = beta_params(0, i * num_body_params + 3);

    // std::cout << robot->getBodyNode(i)->getName() << std::endl;
    robot->getBodyNode(i)->setMass(mi);
    robot->getBodyNode(i)->setLocalCOM(bodyMCOM / mi);
  }
}
void BalanceControl::UpdateReference(const double& forw, const double& spin) {
  // First, set the balancing angle and velocity to zeroes
  refState(0) = refState(1) = 0.0;

  // Set the distance and heading velocities using the joystick input
  refState(3) = forw;
  refState(5) = spin;

  // Integrate the reference positions with the current reference velocities
  refState(2) += dt * refState(3);
  refState(4) += dt * refState(5);
}
void BalanceControl::CancelPositionBuiltup() {
  refState(2) = state(2);
  refState(4) = state(4);
}
void BalanceControl::ForceModeChange(BalanceControl::BalanceMode new_mode) {
  if ((balance_mode_ == BalanceControl::GROUND_LO ||
       balance_mode_ == BalanceControl::GROUND_HI) &&
      (new_mode == BalanceControl::STAND ||
       new_mode == BalanceControl::BAL_LO ||
       new_mode == BalanceControl::BAL_HI)) {
    CancelPositionBuiltup();
  }

  balance_mode_ = new_mode;
}
void BalanceControl::ChangePdGain(int index, double change) {
  pdGains[balance_mode_](index) += change;
}
void BalanceControl::ComputeCurrent(const Eigen::Matrix<double, 6, 1>& K_,
                                    const Eigen::Matrix<double, 6, 1>& error_,
                                    double* control_input) {
  // Calculate individual components of the control input
  u_theta = K_.topLeftCorner<2, 1>().dot(error_.topLeftCorner<2, 1>());
  u_x = K_(2) * error_(2) + K_(3) * error_(3);
  u_spin = -K_.bottomLeftCorner<2, 1>().dot(error_.bottomLeftCorner<2, 1>());
  u_spin = std::max(-30.0, std::min(30.0, u_spin));

  // Calculate current for the wheels
  control_input[0] = std::max(-49.0, std::min(49.0, u_theta + u_x + u_spin));
  control_input[1] = std::max(-49.0, std::min(49.0, u_theta + u_x - u_spin));
}
Eigen::MatrixXd BalanceControl::ComputeLqrGains() {
  // TODO: Get rid of dynamic allocation
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1);
  Eigen::VectorXd B_thWheel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd B_thCOM = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd LQR_Gains = Eigen::VectorXd::Zero(4);

  computeLinearizedDynamics(krang->robot, A, B, B_thWheel, B_thCOM);
  lqr(A, B, lqrQ, lqrR, LQR_Gains);

  const double motor_constant = 12.0 * 0.00706;
  const double gear_ratio = 15;
  LQR_Gains /= (gear_ratio * motor_constant);
  LQR_Gains = lqrHackRatios * LQR_Gains;

  return LQR_Gains;
}
void BalanceControl::BalancingController(double* control_input) {
  // The timer we use for deciding whether krang has stood up and needs to
  // be switched to BAL_LO mode. This timer is supposed to be zero in all
  // other modes excepts STAND mode. This is to ensure that no matter how we
  // transitioned to STAND mode, this timer is zero in the beginning.
  // In STAND mode this timer starts running if robot is balancing. After it
  // crosses the TimerLimit, mode is switched to balancing.
  static int stood_up_timer = 0;
  const int kStoodUpTimerLimit = 100;
  if (balance_mode_ != BalanceControl::STAND) stood_up_timer = 0;

  // Controllers for each mode
  K.setZero();
  switch (balance_mode_) {
    case BalanceControl::GROUND_LO: {
      //  Update Reference
      double forw, spin;
      forw = joystickGains[BalanceControl::GROUND_LO][0] * joystick_forw;
      spin = joystickGains[BalanceControl::GROUND_LO][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error = state - refState;

      // Gains - read fwd and spin only
      K.tail(4) = pdGains[BalanceControl::GROUND_LO].tail(4);

      // Compute the current
      BalanceControl::ComputeCurrent(K, error, &control_input[0]);

      // State Transition - If the waist has been opened too much switch to
      // GROUND_HI mode
      if ((krang->waist->pos[0] - krang->waist->pos[1]) / 2.0 <
          150.0 * M_PI / 180.0) {
        balance_mode_ = BalanceControl::GROUND_HI;
      }

      break;
    }
    case BalanceControl::GROUND_HI: {
      //  Update Reference
      double forw, spin;
      forw = joystickGains[BalanceControl::GROUND_HI][0] * joystick_forw;
      spin = joystickGains[BalanceControl::GROUND_HI][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error = state - refState;

      // Gains - read fwd and spin gains only
      K.tail(4) = pdGains[BalanceControl::GROUND_HI].tail(4);

      // Compute the current
      BalanceControl::ComputeCurrent(K, error, &control_input[0]);

      // State Transitions
      // If in ground Hi mode and waist angle decreases below 150.0 goto
      // groundLo mode
      if ((krang->waist->pos[0] - krang->waist->pos[1]) / 2.0 >
          150.0 * M_PI / 180.0) {
        balance_mode_ = BalanceControl::GROUND_LO;
      }
      break;
    }
    case BalanceControl::STAND: {
      //  Update Reference
      double forw, spin;
      forw = 0.0;
      spin = 0.0;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error = state - refState;

      // Gains - no spinning
      K.head(4) = pdGains[BalanceControl::STAND].head(4);
      if (dynamicLQR == true) {
        Eigen::MatrixXd LQR_Gains;
        LQR_Gains = BalanceControl::ComputeLqrGains();
        K.head(4) = -LQR_Gains;
      }

      // Compute the current
      BalanceControl::ComputeCurrent(K, error, &control_input[0]);

      // State Transition - If stood up go to balancing mode
      if (fabs(state(0)) < (toBalThreshold / 180.0) * M_PI /*0.034*/) {
        stood_up_timer++;
      } else {
        stood_up_timer = 0;
      }
      if (stood_up_timer > kStoodUpTimerLimit) {
        balance_mode_ = BalanceControl::BAL_LO;
      }

      break;
    }
    case BalanceControl::BAL_LO: {
      //  Update Reference
      double forw, spin;
      forw = joystickGains[BalanceControl::BAL_LO][0] * joystick_forw;
      spin = joystickGains[BalanceControl::BAL_LO][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error = state - refState;

      // Gains
      K = pdGains[BalanceControl::BAL_LO];
      if (dynamicLQR == true) {
        Eigen::MatrixXd LQR_Gains;
        LQR_Gains = BalanceControl::ComputeLqrGains();
        K.head(4) = -LQR_Gains;
      }

      // Compute the current
      BalanceControl::ComputeCurrent(K, error, &control_input[0]);

      break;
    }
    case BalanceControl::BAL_HI: {
      //  Update Reference
      double forw, spin;
      forw = joystickGains[BalanceControl::BAL_HI][0] * joystick_forw;
      spin = joystickGains[BalanceControl::BAL_HI][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error = state - refState;

      // Gains
      K = pdGains[BalanceControl::BAL_HI];
      if (dynamicLQR == true) {
        Eigen::MatrixXd LQR_Gains;
        LQR_Gains = BalanceControl::ComputeLqrGains();
        K.head(4) = -LQR_Gains;
      }

      // Compute the current
      BalanceControl::ComputeCurrent(K, error, &control_input[0]);

      break;
    }
    case BalanceControl::SIT: {
      //  Update Reference
      double forw, spin;
      forw = 0.0;
      spin = 0.0;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      const double kImuSitAngle = ((imuSitAngle / 180.0) * M_PI);
      error = state - refState;
      error(0) = krang->imu - kImuSitAngle;

      // Gains - turn off fwd and spin control i.e. only control theta
      K.head(2) = pdGains[BalanceControl::SIT].head(2);

      // Compute the current
      BalanceControl::ComputeCurrent(K, error, &control_input[0]);

      // State Transitions - If sat down switch to Ground Lo Mode
      if (krang->imu < kImuSitAngle) {
        std::cout << "imu (" << krang->imu << ") < limit (" << kImuSitAngle
                  << "):";
        std::cout << "changing to Ground Lo Mode" << std::endl;
        balance_mode_ = BalanceControl::GROUND_LO;
      }

      break;
    }
  }
}

void BalanceControl::Print() {
  std::cout << "\nstate: " << state.transpose() << std::endl;
  std::cout << "com: " << com.transpose() << std::endl;
  std::cout << "WAIST ANGLE: " << krang->waist->pos[0] << std::endl;
  std::cout << "js_forw: " << joystick_forw;
  std::cout << ", js_spin: " << joystick_spin << std::endl;
  std::cout << "refState: " << refState.transpose() << std::endl;
  std::cout << "error: " << error.transpose();
  std::cout << ", imu: " << krang->imu / M_PI * 180.0 << std::endl;
  std::cout << "K: " << K.transpose() << std::endl;
  std::cout << "Mode : " << MODE_STRINGS[balance_mode_] << "      ";
  std::cout << "dt: " << dt << std::endl;
}
void BalanceControl::BalHiLoEvent() {
  if (balance_mode_ == BalanceControl::BAL_LO) {
    balance_mode_ = BalanceControl::BAL_HI;
  } else if (balance_mode_ == BalanceControl::BAL_HI) {
    balance_mode_ = BalanceControl::BAL_LO;
  }
}
void BalanceControl::StandSitEvent() {
  // If in ground mode and state error is not high stand up
  if (balance_mode_ == BalanceControl::GROUND_LO) {
    if (state(0) < 0.0 && error(0) > -10.0 * M_PI / 180.0) {
      balance_mode_ = BalanceControl::STAND;
      CancelPositionBuiltup();
      std::cout << "[MODE] STAND" << std::endl;
    } else {
      std::cout << "[ERR ] Can't stand up!! Bal error too high" << std::endl;
    }
  }

  // If in balLow mode and waist is not too high, sit down
  else if (balance_mode_ == BalanceControl::STAND ||
           balance_mode_ == BalanceControl::BAL_LO) {
    if ((krang->waist->pos[0] - krang->waist->pos[1]) / 2.0 >
        150.0 * M_PI / 180.0) {
      balance_mode_ = BalanceControl::SIT;
      std::cout << "[MODE] SIT " << std::endl;
    } else {
      std::cout << "[ERR ] Can't sit down, Waist is too high! " << std::endl;
    }
  }
}
void BalanceControl::SetFwdInput(double forw) { joystick_forw = forw; }
void BalanceControl::SetSpinInput(double spin) { joystick_spin = spin; }
