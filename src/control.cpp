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
 * @date Jan 22, 2019
 * @brief Implements mpc along with the legacy balancing control functions
 */

#include "balancing/control.h"

#include <algorithm>  // std::max(), std::min()
#include <cmath>      // atan2, tan
#include <iostream>   // std::cout, std::endl
#include <string>     // std::string

#include <amino/time.h>  // aa_tm: _now(), _timespec2sec(), _sub()
#include <Eigen/Eigen>  // Eigen:: MatrixXd, VectorXd, Vector3d, Matrix<double, #, #>
#include <dart/dart.hpp>             // dart::dynamics::SkeletonPtr
#include <dart/utils/urdf/urdf.hpp>  // dart::utils::DartLoader
#include <kore.hpp>                  // Krang::Hardware

#include "balancing/adrc.hpp"            // computeLinearizedDynamics()
#include "balancing/balancing_config.h"  // BalancingConfig
#include "balancing/file_ops.hpp"        // readInputFileAsMatrix()
#include "balancing/lqr.hpp"             // lqr()
#include "balancing/mpc.h"               // Mpc

//============================================================================
const char BalanceControl::BAL_MODE_STRINGS[][16] = {
    "Ground Lo", "Stand", "Sit", "Bal Lo", "Bal Hi", "Ground Hi", "MPC"};
//============================================================================
BalanceControl::BalanceControl(Krang::Hardware* krang,
                               dart::dynamics::SkeletonPtr robot,
                               BalancingConfig& params)
    : krang_(krang),
      robot_(robot),
      mpc_("/usr/local/share/krang/balancing/cfg/mpc_params.cfg") {
  // LQR Gains
  dynamic_lqr_ = params.dynamicLQR;
  lqrQ_ = params.lqrQ;
  lqrR_ = params.lqrR;

  // PD Gains for all modes
  pd_gains_list_[BalanceControl::GROUND_LO] = params.pdGainsGroundLo;
  pd_gains_list_[BalanceControl::GROUND_HI] = params.pdGainsGroundHi;
  pd_gains_list_[BalanceControl::STAND] = params.pdGainsStand;
  pd_gains_list_[BalanceControl::SIT] = params.pdGainsSit;
  pd_gains_list_[BalanceControl::BAL_LO] = params.pdGainsBalLo;
  pd_gains_list_[BalanceControl::BAL_HI] = params.pdGainsBalHi;

  // Joystick Gains for all modes
  for (int i = 0; i < 2; i++) {
    joystick_gains_list_[BalanceControl::GROUND_LO][i] =
        params.joystickGainsGroundLo[i];
    joystick_gains_list_[BalanceControl::GROUND_HI][i] =
        params.joystickGainsGroundHi[i];
    joystick_gains_list_[BalanceControl::STAND][i] =
        params.joystickGainsStand[i];
    joystick_gains_list_[BalanceControl::SIT][i] = params.joystickGainsSit[i];
    joystick_gains_list_[BalanceControl::BAL_LO][i] =
        params.joystickGainsBalLo[i];
    joystick_gains_list_[BalanceControl::BAL_HI][i] =
        params.joystickGainsBalHi[i];
  }

  // Parameters used for generating internal events
  to_bal_threshold_ = params.toBalThreshold;
  imu_sit_angle_ = params.imuSitAngle;

  // Initial values
  balance_mode_ = BalanceControl::GROUND_LO;
  pd_gains_ = pd_gains_list_[BalanceControl::GROUND_LO];
  ref_state_.setZero();
  state_.setZero();
  error_.setZero();
  joystick_forw = 0.0;
  joystick_spin = 0.0;

  // LQR Hack Ratios
  Eigen::VectorXd lqrGains = Eigen::VectorXd::Zero(4);
  lqr_hack_ratios_ = Eigen::Matrix<double, 4, 4>::Identity();
  lqrGains = BalanceControl::ComputeLqrGains();
  for (int i = 0; i < lqr_hack_ratios_.cols(); i++) {
    lqr_hack_ratios_(i, i) =
        pd_gains_list_[BalanceControl::STAND](i) / -lqrGains(i);
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

  // Cloning the robot so that mutex sharing is not necessary among
  // threads
  mpc_.ddp_robot_ = robot_->clone();

  // time
  t_prev_ = aa_tm_now();
}

//============================================================================
double BalanceControl::ElapsedTimeSinceLastCall() {
  t_now_ = aa_tm_now();
  dt_ = (double)aa_tm_timespec2sec(aa_tm_sub(t_now_, t_prev_));
  t_prev_ = t_now_;

  return dt_;
}

//============================================================================
void BalanceControl::UpdateState() {
  // Read motor encoders, imu and ft and update dart skeleton
  krang_->updateSensors(dt_);

  // Update robot pose variable shared with the ddp thread
  mpc_.robot_pose_mutex_.lock();
  mpc_.robot_pose_ = robot_->getPositions();
  mpc_.robot_pose_mutex_.unlock();

  // Calculate the COM Using Skeleton
  com_ = robot_->getCOM() - robot_->getPositions().segment(3, 3);

  // Update the state (note for amc we are reversing the effect of the motion of
  // the upper body) State are theta, dtheta, x, dx, psi, dpsi
  state_(0) = atan2(com_(0), com_(2));  // - 0.3 * M_PI / 180.0;;
  state_(1) = krang_->imuSpeed;
  state_(2) = (krang_->amc->pos[0] + krang_->amc->pos[1]) / 2.0 + krang_->imu;
  state_(3) =
      (krang_->amc->vel[0] + krang_->amc->vel[1]) / 2.0 + krang_->imuSpeed;
  state_(4) = (krang_->amc->pos[1] - krang_->amc->pos[0]) / 2.0;
  state_(5) = (krang_->amc->vel[1] - krang_->amc->vel[0]) / 2.0;

  // Making adjustment in com to make it consistent with the hack above for
  // state(0)
  com_(0) = com_(2) * tan(state_(0));

  // copy of state for the ddp thread
  mpc_.ddp_bal_state_mutex_.lock();
  mpc_.ddp_bal_state_ << state_;
  mpc_.ddp_bal_state_mutex_.unlock();
}

//============================================================================
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
    robot_->getBodyNode(i)->setMass(mi);
    robot_->getBodyNode(i)->setLocalCOM(bodyMCOM / mi);
  }
}

//============================================================================
void BalanceControl::UpdateReference(const double& forw, const double& spin) {
  // First, set the balancing angle and velocity to zeroes
  ref_state_(0) = ref_state_(1) = 0.0;

  // Set the distance and heading velocities using the joystick input
  ref_state_(3) = forw;
  ref_state_(5) = spin;

  // Integrate the reference positions with the current reference velocities
  ref_state_(2) += dt_ * ref_state_(3);
  ref_state_(4) += dt_ * ref_state_(5);
}

//============================================================================
void BalanceControl::CancelPositionBuiltup() {
  ref_state_(2) = state_(2);
  ref_state_(4) = state_(4);
}

//============================================================================
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

//============================================================================
void BalanceControl::ChangePdGain(int index, double change) {
  pd_gains_list_[balance_mode_](index) += change;
}

//============================================================================
void BalanceControl::ComputeCurrent(const Eigen::Matrix<double, 6, 1>& pd_gain,
                                    const Eigen::Matrix<double, 6, 1>& error,
                                    double* control_input) {
  // Calculate individual components of the control input
  u_theta_ = pd_gain.topLeftCorner<2, 1>().dot(error.topLeftCorner<2, 1>());
  u_x_ = pd_gain(2) * error(2) + pd_gain(3) * error(3);
  u_spin_ =
      -pd_gain.bottomLeftCorner<2, 1>().dot(error.bottomLeftCorner<2, 1>());
  u_spin_ = std::max(-30.0, std::min(30.0, u_spin_));

  // Calculate current for the wheels
  control_input[0] = std::max(-49.0, std::min(49.0, u_theta_ + u_x_ + u_spin_));
  control_input[1] = std::max(-49.0, std::min(49.0, u_theta_ + u_x_ - u_spin_));
}

//============================================================================
Eigen::MatrixXd BalanceControl::ComputeLqrGains() {
  // TODO: Get rid of dynamic allocation
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1);
  Eigen::VectorXd B_thWheel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd B_thCOM = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd LQR_Gains = Eigen::VectorXd::Zero(4);

  computeLinearizedDynamics(krang_->robot, A, B, B_thWheel, B_thCOM);
  lqr(A, B, lqrQ_, lqrR_, LQR_Gains);

  const double motor_constant = 12.0 * 0.00706;
  const double gear_ratio = 15;
  LQR_Gains /= (gear_ratio * motor_constant);
  LQR_Gains = lqr_hack_ratios_ * LQR_Gains;

  return LQR_Gains;
}

//============================================================================
void BalanceControl::BalancingController(double* control_input) {
  // The timer we use for deciding whether krang_ has stood up and needs to
  // be switched to BAL_LO mode. This timer is supposed to be zero in all
  // other modes excepts STAND mode. This is to ensure that no matter how we
  // transitioned to STAND mode, this timer is zero in the beginning.
  // In STAND mode this timer starts running if robot is balancing. After it
  // crosses the TimerLimit, mode is switched to balancing.
  static int stood_up_timer = 0;
  const int kStoodUpTimerLimit = 100;
  if (balance_mode_ != BalanceControl::STAND) stood_up_timer = 0;

  // Controllers for each mode
  pd_gains_.setZero();
  switch (balance_mode_) {
    case BalanceControl::GROUND_LO: {
      //  Update Reference
      double forw, spin;
      forw = joystick_gains_list_[BalanceControl::GROUND_LO][0] * joystick_forw;
      spin = joystick_gains_list_[BalanceControl::GROUND_LO][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error_ = state_ - ref_state_;

      // Gains - read fwd and spin only
      pd_gains_.tail(4) = pd_gains_list_[BalanceControl::GROUND_LO].tail(4);

      // Compute the current
      BalanceControl::ComputeCurrent(pd_gains_, error_, &control_input[0]);

      // State Transition - If the waist has been opened too much switch to
      // GROUND_HI mode
      if ((krang_->waist->pos[0] - krang_->waist->pos[1]) / 2.0 <
          150.0 * M_PI / 180.0) {
        balance_mode_ = BalanceControl::GROUND_HI;
      }

      break;
    }
    case BalanceControl::GROUND_HI: {
      //  Update Reference
      double forw, spin;
      forw = joystick_gains_list_[BalanceControl::GROUND_HI][0] * joystick_forw;
      spin = joystick_gains_list_[BalanceControl::GROUND_HI][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error_ = state_ - ref_state_;

      // Gains - read fwd and spin gains only
      pd_gains_.tail(4) = pd_gains_list_[BalanceControl::GROUND_HI].tail(4);

      // Compute the current
      BalanceControl::ComputeCurrent(pd_gains_, error_, &control_input[0]);

      // State Transitions
      // If in ground Hi mode and waist angle decreases below 150.0 goto
      // groundLo mode
      if ((krang_->waist->pos[0] - krang_->waist->pos[1]) / 2.0 >
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
      error_ = state_ - ref_state_;

      // Gains - no spinning
      pd_gains_.head(4) = pd_gains_list_[BalanceControl::STAND].head(4);
      if (dynamic_lqr_ == true) {
        Eigen::MatrixXd LQR_Gains;
        LQR_Gains = BalanceControl::ComputeLqrGains();
        pd_gains_.head(4) = -LQR_Gains;
      }

      // Compute the current
      BalanceControl::ComputeCurrent(pd_gains_, error_, &control_input[0]);

      // State Transition - If stood up go to balancing mode
      if (fabs(state_(0)) < (to_bal_threshold_ / 180.0) * M_PI /*0.034*/) {
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
      forw = joystick_gains_list_[BalanceControl::BAL_LO][0] * joystick_forw;
      spin = joystick_gains_list_[BalanceControl::BAL_LO][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error_ = state_ - ref_state_;

      // Gains
      pd_gains_ = pd_gains_list_[BalanceControl::BAL_LO];
      if (dynamic_lqr_ == true) {
        Eigen::MatrixXd LQR_Gains;
        LQR_Gains = BalanceControl::ComputeLqrGains();
        pd_gains_.head(4) = -LQR_Gains;
      }

      // Compute the current
      BalanceControl::ComputeCurrent(pd_gains_, error_, &control_input[0]);

      break;
    }
    case BalanceControl::BAL_HI: {
      //  Update Reference
      double forw, spin;
      forw = joystick_gains_list_[BalanceControl::BAL_HI][0] * joystick_forw;
      spin = joystick_gains_list_[BalanceControl::BAL_HI][1] * joystick_spin;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      error_ = state_ - ref_state_;

      // Gains
      pd_gains_ = pd_gains_list_[BalanceControl::BAL_HI];
      if (dynamic_lqr_ == true) {
        Eigen::MatrixXd LQR_Gains;
        LQR_Gains = BalanceControl::ComputeLqrGains();
        pd_gains_.head(4) = -LQR_Gains;
      }

      // Compute the current
      BalanceControl::ComputeCurrent(pd_gains_, error_, &control_input[0]);

      break;
    }
    case BalanceControl::SIT: {
      //  Update Reference
      double forw, spin;
      forw = 0.0;
      spin = 0.0;
      BalanceControl::UpdateReference(forw, spin);

      // Calculate state Error
      const double kImuSitAngle = ((imu_sit_angle_ / 180.0) * M_PI);
      error_ = state_ - ref_state_;
      error_(0) = krang_->imu - kImuSitAngle;

      // Gains - turn off fwd and spin control i.e. only control theta
      pd_gains_.head(2) = pd_gains_list_[BalanceControl::SIT].head(2);

      // Compute the current
      BalanceControl::ComputeCurrent(pd_gains_, error_, &control_input[0]);

      // State Transitions - If sat down switch to Ground Lo Mode
      if (krang_->imu < kImuSitAngle) {
        std::cout << "imu (" << krang_->imu << ") < limit (" << kImuSitAngle
                  << "):";
        std::cout << "changing to Ground Lo Mode" << std::endl;
        balance_mode_ = BalanceControl::GROUND_LO;
      }

      break;
    }
    case BalanceControl::MPC: {
      // Check exit condition
      struct timespec t_now = aa_tm_now();
      double time_now = (double)aa_tm_timespec2sec(t_now);
      double init_time = mpc_.GetInitTime();
      if (time_now >= init_time + mpc_.param_.ddp_.final_time_) {
        // Change to idle mode and get out
        balance_mode_ = previous_balance_mode_;
        break;
      }

      // Current step
      int current_step = floor((time_now - init_time)/mpc_.param_.mpc_.dt_);

      /*
      // =============== Read mpc control step
      bool mpc_reading_done = false;
      Control u;
      while (!mpc_reading_done) {
        if (pthread_mutex_trylock(&g_mpc_trajectory_main_mutex) == 0) {
          u = g_mpc_trajectory_main.col(
              MPCStepIdx);  // Using counter to get the correct reference
          pthread_mutex_unlock(&g_mpc_trajectory_main_mutex);
          mpc_reading_done = true;
        } else if (pthread_mutex_trylock(&g_mpc_trajectory_backup_mutex) ==
                   0) {
          u = g_mpc_trajectory_main.col(
              MPCStepIdx);  // Using counter to get the correct reference
          pthread_mutex_unlock(&g_mpc_trajectory_backup_mutex);
          mpc_reading_done = true;
        }
      }

      // =============== Spin Torque tau_0
      double tau_0 = u(1);

      // =============== Forward Torque tau_1

      // ddthref from High-level Control
      double ddth = u(0);

      // Get dynamics object
      DDPDynamics* mpc_dynamics = getDynamics(g_threeDOF);

      // current state
      pthread_mutex_lock(&g_state_mutex);
      pthread_mutex_lock(&g_augstate_mutex);
      State cur_state;
      cur_state << 0.25 * g_state(2) - g_xInit, g_state(4) - g_psiInit,
          g_state(0), 0.25 * g_state(3), g_state(5), g_state(1),
          g_augstate(0), g_augstate(1);
      pthread_mutex_unlock(&g_state_mutex);
      pthread_mutex_unlock(&g_augstate_mutex);

      State xdot = mpc_dynamics->f(cur_state, u);
      double ddx = xdot(3);
      double ddpsi = xdot(4);
      Eigen::Vector3d ddq;
      ddq << ddx, ddpsi, ddth;

      // dq
      Eigen::Vector3d dq = cur_state.segment(3, 3);

      // A, C, Q and Gamma_fric
      c_forces dy_forces = mpc_dynamics->dynamic_forces(cur_state, u);

      // tau_1
      double tau_1 = dy_forces.A.block<1, 3>(2, 0) * ddq;
      tau_1 += dy_forces.C.block<1, 3>(2, 0) * dq;
      tau_1 += dy_forces.Q(2);
      tau_1 -= dy_forces.Gamma_fric(2);

      // =============== Wheel Torques
      double tau_L = -0.5 * (tau_1 + tau_0);
      double tau_R = -0.5 * (tau_1 - tau_0);

      // =============== Torque to current conversion

      // Motor Constant
      // page 2, row "BLY343D-24V-2000" of:
      // https://www.anaheimautomation.com/manuals/brushless/L010350%20-%20BLY34%20Series%20Product%20Sheet.pdf
      double km = 12.0 * 0.00706;  // 12 (oz-in/A) * 0.00706 (Nm/oz-in)

      // Gear Ratio
      // page 2, row "GBPH-0902-NS-015-xxxxx-yyy" of:
      // https://www.anaheimautomation.com/manuals/gearbox/L010455%20-%20GBPH-090x-NS%20Series%20Spec%20Sheet.pdf
      double GR = 15;

      double input[2];
      input[0] = min(49.5, max(-49.5, tau_L / GR / km));
      input[1] = min(49.5, max(-49.5, tau_R / GR / km));
      lastUleft = input[0], lastUright = input[1];

      // =============== Set the Motor Currents
      controlWheels(debug, input);
      */

      break;
    }
  }
}

//============================================================================
void BalanceControl::Print() {
  std::cout << "\nstate: " << state_.transpose() << std::endl;
  std::cout << "com: " << com_.transpose() << std::endl;
  std::cout << "WAIST ANGLE: " << krang_->waist->pos[0] << std::endl;
  std::cout << "js_forw: " << joystick_forw;
  std::cout << ", js_spin: " << joystick_spin << std::endl;
  std::cout << "refState: " << ref_state_.transpose() << std::endl;
  std::cout << "error: " << error_.transpose();
  std::cout << ", imu: " << krang_->imu / M_PI * 180.0 << std::endl;
  std::cout << "PD Gains: " << pd_gains_.transpose() << std::endl;
  std::cout << "Mode : " << BAL_MODE_STRINGS[balance_mode_] << " - ";
  std::cout << mpc_.DDP_MODE_STRINGS[mpc_.GetDdpMode()] << "     ";
  std::cout << "dt: " << dt_ << std::endl;
}

//============================================================================
void BalanceControl::BalHiLoEvent() {
  if (balance_mode_ == BalanceControl::BAL_LO) {
    balance_mode_ = BalanceControl::BAL_HI;
  } else if (balance_mode_ == BalanceControl::BAL_HI) {
    balance_mode_ = BalanceControl::BAL_LO;
  }
}

//============================================================================
void BalanceControl::StandSitEvent() {
  // If in ground mode and state error is not high stand up
  if (balance_mode_ == BalanceControl::GROUND_LO) {
    if (state_(0) < 0.0 && error_(0) > -10.0 * M_PI / 180.0) {
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
    if ((krang_->waist->pos[0] - krang_->waist->pos[1]) / 2.0 >
        150.0 * M_PI / 180.0) {
      balance_mode_ = BalanceControl::SIT;
      std::cout << "[MODE] SIT " << std::endl;
    } else {
      std::cout << "[ERR ] Can't sit down, Waist is too high! " << std::endl;
    }
  }
}

//============================================================================
void BalanceControl::SetFwdInput(double forw) { joystick_forw = forw; }

//============================================================================
void BalanceControl::SetSpinInput(double spin) { joystick_spin = spin; }
