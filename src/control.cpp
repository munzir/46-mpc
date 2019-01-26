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
#include <mutex>      // std::mutex
#include <string>     // std::string
#include <thread>     // std::thread

#include <amino/time.h>  // aa_tm: _now(), _timespec2sec(), _sub()
#include <Eigen/Eigen>  // Eigen:: MatrixXd, VectorXd, Vector3d, Matrix<double, #, #>
#include <dart/dart.hpp>             // dart::dynamics::SkeletonPtr
#include <dart/utils/urdf/urdf.hpp>  // dart::utils::DartLoader
#include <kore.hpp>                  // Krang::Hardware

#include "balancing/adrc.hpp"            // computeLinearizedDynamics()
#include "balancing/balancing_config.h"  // BalancingConfig
#include "balancing/ddp_objects.h"       // TwipDynamics...
#include "balancing/file_ops.hpp"        // readInputFileAsMatrix()
#include "balancing/lqr.hpp"             // lqr()

//============================================================================
const char BalanceControl::BAL_MODE_STRINGS[][16] = {
    "Ground Lo", "Stand", "Sit", "Bal Lo", "Bal Hi", "Ground Hi", "MPC"};
const char BalanceControl::DDP_MODE_STRINGS[][32] = {
    "DDP Idle", "DDP Compute Traj", "DDP Traj OK?", "DDP for MPC"};

//============================================================================
BalanceControl::BalanceControl(Krang::Hardware* krang,
                               dart::dynamics::SkeletonPtr robot,
                               BalancingConfig& params)
    : krang_(krang), robot_(robot) {
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

  // Launch the DDP thread
  ddp_thread_run_ = true;
  ddp_thread_ = new std::thread(&BalanceControl::DdpThread, this);
  ddp_robot_ = robot_->clone();
  dart::utils::DartLoader loader;
  three_dof_robot_ = loader.parseSkeleton(params.threeDofUrdfpath);
  three_dof_robot_->getJoint(0)->setDampingCoefficient(0, 0.5);
  three_dof_robot_->getJoint(1)->setDampingCoefficient(0, 0.5);
  SetDdpMode(DDP_IDLE);

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

//============================================================================
void BalanceControl::UpdateState() {
  // Read motor encoders, imu and ft and update dart skeleton
  krang_->updateSensors(dt_);

  // Update robot pose variable shared with the ddp thread
  robot_pose_mutex_.lock();
  robot_pose_ = robot_->getPositions();
  robot_pose_mutex_.unlock();

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
  ddp_bal_state_mutex_.lock();
  ddp_bal_state_ << state_;
  ddp_bal_state_mutex_.unlock();
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
  std::cout << DDP_MODE_STRINGS[GetDdpMode()] << "     ";
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

//============================================================================
BalanceControl::DdpMode BalanceControl::GetDdpMode() {
  ddp_mode_mutex_.lock();
  DdpMode ddp_mode = ddp_mode_;
  ddp_mode_mutex_.unlock();
  return ddp_mode;
}

//============================================================================
void BalanceControl::SetDdpMode(DdpMode ddp_mode) {
  ddp_mode_mutex_.lock();
  ddp_mode_ = ddp_mode;
  ddp_mode_mutex_.unlock();
}

//============================================================================
// Update the position and velocity of a 3dof dart model with that of the global
// krang model
void BalanceControl::UpdateThreeDof(
    dart::dynamics::SkeletonPtr& robot,
    dart::dynamics::SkeletonPtr& three_dof_robot) {
  // Body Mass
  double full_mass = robot->getMass();
  double lwheel_mass = robot->getBodyNode("LWheel")->getMass();
  double rwheel_mass = robot->getBodyNode("RWheel")->getMass();
  double body_mass = full_mass - lwheel_mass - rwheel_mass;

  // Body CoM
  Eigen::Vector3d body_com;
  dart::dynamics::Frame* base_frame = robot->getBodyNode("Base");
  body_com = (full_mass * robot->getCOM(base_frame) -
              lwheel_mass * robot->getBodyNode("LWheel")->getCOM(base_frame) -
              rwheel_mass * robot->getBodyNode("RWheel")->getCOM(base_frame)) /
             (full_mass - lwheel_mass - rwheel_mass);

  // Body inertia (axis)
  double link_mass;
  Eigen::Matrix3d link_inertia;
  Eigen::Matrix3d body_inertia = Eigen::Matrix3d::Zero();
  double ixx, iyy, izz, ixy, ixz, iyz;
  Eigen::Matrix3d rot;
  Eigen::Vector3d t;  // translation vector
  Eigen::Matrix3d
      t_mat;  // matrix from translation vector used for parallel axis theorem
  dart::dynamics::BodyNodePtr link;
  int num_links = robot->getNumBodyNodes();

  for (int i = 0; i < num_links; i++) {
    if (i == 1 || i == 2) continue;  // Skip wheels
    link = robot->getBodyNode(i);
    link->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = link->getTransform(base_frame).rotation();
    t = robot->getCOM(base_frame) -
        link->getCOM(base_frame);  // Position vector from local COM to body COM
                                   // expressed in base frame
    link_mass = link->getMass();
    link_inertia << ixx, ixy, ixz,  // Inertia tensor of the body around its CoM
                                    // expressed in body frame
        ixy, iyy, iyz, ixz, iyz, izz;
    link_inertia = rot * link_inertia *
                   rot.transpose();  // Inertia tensor of the body around
                                     // its CoM expressed in base frame
    t_mat << (t(1) * t(1) + t(2) * t(2)), (-t(0) * t(1)), (-t(0) * t(2)),
        (-t(0) * t(1)), (t(0) * t(0) + t(2) * t(2)), (-t(1) * t(2)),
        (-t(0) * t(2)), (-t(1) * t(2)), (t(0) * t(0) + t(1) * t(1));
    link_inertia = link_inertia + link_mass * t_mat;  // Parallel Axis Theorem
    body_inertia += link_inertia;
  }

  // Aligning threeDOF base frame to have the y-axis pass through the CoM
  double th = atan2(body_com(2), body_com(1));
  rot << 1, 0, 0, 0, cos(th), sin(th), 0, -sin(th), cos(th);
  body_com = rot * body_com;
  body_inertia = rot * body_inertia * rot.transpose();

  // Set the 3 DOF robot parameters
  three_dof_robot_mutex_.lock();
  three_dof_robot->getBodyNode("Base")->setMomentOfInertia(
      body_inertia(0, 0), body_inertia(1, 1), body_inertia(2, 2),
      body_inertia(0, 1), body_inertia(0, 2), body_inertia(1, 2));
  three_dof_robot->getBodyNode("Base")->setLocalCOM(body_com);
  three_dof_robot->getBodyNode("Base")->setMass(body_mass);

  // Update 3DOF state
  // get positions
  Eigen::Matrix3d base_rot =
      robot->getBodyNode("Base")->getTransform().rotation();
  base_rot = base_rot * rot.transpose();
  Eigen::AngleAxisd aa(base_rot);
  Eigen::Matrix<double, 8, 1> q, dq;

  q << aa.angle() * aa.axis(), robot->getPositions().segment(3, 5);
  three_dof_robot->setPositions(q);
  // TODO: When joints are unlocked qBody1 of the 3DOF (= dth = COM angular
  // speed) is not the same as qBody1 of the full robot
  dq << rot * robot->getVelocities().head(3),
      rot * robot->getVelocities().segment(3, 3),
      robot->getVelocities().segment(6, 2);
  three_dof_robot->setVelocities(dq);
  three_dof_robot_mutex_.unlock();
}

//============================================================================
TwipDynamics<double>* BalanceControl::DartSkeletonToTwipDynamics(
    SkeletonPtr& three_dof_robot) {
  // Wheel radius, distance between wheels and gravitational acceleration
  TwipDynamics<double>::Parameters p;
  p.R = 0.25;
  p.L = 0.68;
  p.g = 9.800000e+00;

  // Wheel mass
  three_dof_robot_mutex_.lock();
  p.mw = three_dof_robot->getBodyNode("LWheel")->getMass();

  // Wheel inertia
  double ixx, iyy, izz, ixy, ixz, iyz;
  three_dof_robot->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy,
                                                             ixz, iyz);
  p.YYw = ixx;
  p.ZZw = izz;
  p.XXw = iyy;  // Wheel frame of reference in ddp dynamic model is different
                // from the one in DART

  // Body mass
  p.m_1 = three_dof_robot->getBodyNode("Base")->getMass();

  // Body CoM
  dart::dynamics::Frame* base_frame = three_dof_robot->getBodyNode("Base");
  Eigen::Vector3d com =
      three_dof_robot->getBodyNode("Base")->getCOM(base_frame);
  p.MX_1 = p.m_1 * com(0);
  p.MY_1 = p.m_1 * com(1);
  p.MZ_1 = p.m_1 * com(2);

  // Body inertia
  three_dof_robot->getBodyNode("Base")->getMomentOfInertia(ixx, iyy, izz, ixy,
                                                           ixz, iyz);
  Eigen::Vector3d s = -com;  // Position vector from local COM to body COM
                             // expressed in base frame
  Eigen::Matrix3d inertia;
  inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz,
      izz;  // Inertia tensor of the body around its CoM expressed in body frame
  Eigen::Matrix3d tMat;
  tMat << (s(1) * s(1) + s(2) * s(2)), (-s(0) * s(1)), (-s(0) * s(2)),
      (-s(0) * s(1)), (s(0) * s(0) + s(2) * s(2)), (-s(1) * s(2)),
      (-s(0) * s(2)), (-s(1) * s(2)), (s(0) * s(0) + s(1) * s(1));
  inertia = inertia + p.m_1 * tMat;  // Parallel Axis Theorem
  p.XX_1 = inertia(0, 0);
  p.YY_1 = inertia(1, 1);
  p.ZZ_1 = inertia(2, 2);
  p.XY_1 = inertia(0, 1);
  p.YZ_1 = inertia(1, 2);
  p.XZ_1 = inertia(0, 2);

  // Friction
  p.fric_1 = three_dof_robot->getJoint(0)->getDampingCoefficient(
      0);  // Assuming both joints have same friction coeff (Please make sure
           // that is true)
  three_dof_robot_mutex_.unlock();

  // Create and return TwipDynamics object
  return new TwipDynamics<double>(p);
}

//============================================================================
void BalanceControl::DdpThread() {
  std::cout << "Entering DDP Thread ..." << std::endl;

  bool run = true;
  while (run) {
    // Implementation for each DDP mode
    switch (GetDdpMode()) {
      case DDP_IDLE: {
        break;
      }
      case DDP_COMPUTE_TRAJ: {
        // Lock mutexes
        ddp_bal_state_mutex_.lock();
        ddp_init_heading_mutex_.lock();
        ddp_augmented_state_mutex_.lock();

        // Define initial heading for computation of future mpc states
        ddp_init_heading_.distance_ = 0.25 * ddp_bal_state_(2);
        ddp_init_heading_.direction_ = ddp_bal_state_(4);

        // Reset augmented state
        ddp_augmented_state_.x0_ = 0.0;
        ddp_augmented_state_.y0_ = 0.0;

        // Initial state for ddp trajectory computation
        TwipDynamics<double>::State x0;
        x0 << 0.25 * ddp_bal_state_(2) - ddp_init_heading_.distance_,
            ddp_bal_state_(4) - ddp_init_heading_.direction_, ddp_bal_state_(0),
            0.25 * ddp_bal_state_(3), ddp_bal_state_(5), ddp_bal_state_(1),
            ddp_augmented_state_.x0_, ddp_augmented_state_.y0_;

        // Unlock mutexes
        ddp_augmented_state_mutex_.unlock();
        ddp_init_heading_mutex_.unlock();
        ddp_bal_state_mutex_.unlock();

        // Get the dynamic model for ddp
        robot_pose_mutex_.lock();
        ddp_robot_->setPositions(robot_pose_);
        robot_pose_mutex_.unlock();
        UpdateThreeDof(ddp_robot_, three_dof_robot_);
        TwipDynamics<double>* ddp_dynamics =
            DartSkeletonToTwipDynamics(three_dof_robot_);
        break;
      }
      case DDP_TRAJ_OK: {
        break;
      }
      case DDP_FOR_MPC: {
        break;
      }
    }

    // Loop will exit if ddp_thread_run_ is reset
    ddp_thread_run_mutex_.lock();
    run = ddp_thread_run_;
    ddp_thread_run_mutex_.unlock();
  }
  std::cout << "Exiting DDP Thread ..." << std::endl;
}

//============================================================================
void BalanceControl::Destroy() {
  ddp_thread_run_mutex_.lock();
  ddp_thread_run_ = false;
  ddp_thread_run_mutex_.unlock();
  ddp_thread_->join();
  delete ddp_thread_;
}
