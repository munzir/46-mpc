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
 * @file control.h
 * @author Munzir Zafar
 * @date Jan 22, 2019
 * @brief Header file for control.cpp that implements mpc along with the
 * legacy balancing control functions
 */

#ifndef KRANG_BALANCING_CONTROL_H_
#define KRANG_BALANCING_CONTROL_H_

#include <Eigen/Eigen>    // Eigen::MatrixXd, Eigen::Matrix<double, #, #>
#include <dart/dart.hpp>  // dart::dynamics::SkeletonPtr
#include <kore.hpp>       // Krang::Hardware

#include "balancing/balancing_config.h"  // BalancingConfig
#include "balancing/mpc.h"               // Mpc

class BalanceControl {
 public:
  BalanceControl(Krang::Hardware* krang_, dart::dynamics::SkeletonPtr robot_,
                 BalancingConfig& params);
  // The states of our state machine. We use the name "mode" instead of "state"
  // because state is already being used to name the state of the wheeled
  // inverted pendulum dynamics
  enum BalanceMode {
    GROUND_LO = 0,
    STAND,
    SIT,
    BAL_LO,
    BAL_HI,
    GROUND_HI,
    MPC,
    NUM_BAL_MODES
  };
  static const char BAL_MODE_STRINGS[][16];

  // Returns the time in seconds since last call to this function
  // For the first call, returns the time elapsed since the call to the
  // constructor
  double ElapsedTimeSinceLastCall();

  // Reads the sensors of the robot and updates the state of the wheeled
  // inverted pendulum. Involves computation of the center of mass
  void UpdateState();

  // Sets reference positions for heading and spin the current values
  void CancelPositionBuiltup();

  // Forces the state machine to go to the specified mode. Calls
  // CancelPositionBuiltup() if there is transition from Ground modes to other
  // modes
  void ForceModeChange(BalanceMode new_mode);

  // The main controller logic. Implements control logic as applicable to the
  // current mode of the state machine
  void BalancingController(double* control_input);

  // Change a gain among the current pd_gains_
  // index: represents the targeted gain
  // change: the amount by which to change the gain
  void ChangePdGain(int index, double change);

  // Dump relevant info on the screen
  void Print();

  // Triggers Stand/Sit event. If in Ground Lo mode, switches to Stand mode. If
  // in Bal Lo/Stand/Mpc mode, switches to Sit mode. If some guards are
  // satisfied.
  // Also, switches to DDP_IDLE if it wasn't already DDP_IDLE
  void StandSitEvent();

  // Triggers Bal Hi/Lo event. If in Bal Lo mode, switches to Bal Hi mode and
  // vice versa
  void BalHiLoEvent();

  // Switches from DDP_IDLE to DDP_COMPUTE_TRAJ mode. Guard: balance_mode_
  // should be BAL_LO or BAL_HI
  void StartMpcEvent();

  // If in DDP_TRAJ_OK mode, switches balance_mode_ to MPC and mpc_.mode_
  // to DDP_FOR_MPC
  void UserAcceptsTrajectoryEvent();

  // If in DDP_TRAJ_OK mode, switches to DDP_COMPUTE_TRAJ mode
  void UserDemandsRecomputationEvent();

  // If balance_mode_ is MPC, switches to previous_balance_mode_
  // And switches mpc_.mode_ to DDP_IDLE
  void StopMpcEvent();

  // Sets the forward speed control reference
  void SetFwdInput(double forw);

  // Sets spin speed control reference
  void SetSpinInput(double spin);

 private:
  // Set parameters in the model used to compute CoM
  // beta_params: list of all CoM parameters for each body
  // num_body_params: how many parameters per body
  void SetComParameters(Eigen::MatrixXd beta_params, int num_body_params);

  // Set the forward and spin pos/vel references based on the respective control
  // references
  void UpdateReference(const double& forw, const double& spin);

  // Based on the pd_gain and error, compute the wheel currents
  // pd_gain: Input parameter
  // error: Input parameter
  // control_input: Output parameter. Array of size 2, with first element
  // representing left wheel and second element representing right wheel
  void ComputeCurrent(const Eigen::Matrix<double, 6, 1>& pd_gain,
                      const Eigen::Matrix<double, 6, 1>& error,
                      double* control_input);

  // Based on the current state of the full robot, computes linearized dynamics
  // of the simplified robot (i.e. the wheeled inverted pendulum) and then
  // computes the LQR gains on the linearized dynamics using costs lqrQ_ and
  // lqrR_ to return a 4-element vector comprising the LQR gains for theta,
  // dtheta, x, dx respectively
  Eigen::MatrixXd ComputeLqrGains();

 private:
  BalanceMode
      balance_mode_;  // Current mode of the balancing thread state machine
  Eigen::Matrix<double, 4, 4>
      lqr_hack_ratios_;  // gains_that_work/computed_lqr_gains
  Eigen::Matrix<double, 6, 1> pd_gains_, ref_state_, state_,
      error_;  // state: th, dth, forw, dforw, spin, dspin
  Eigen::Matrix<double, 6, 1>
      pd_gains_list_[NUM_BAL_MODES];  // fixed pd gains for each mode
                                      // specified in the config file
  double joystick_gains_list_[NUM_BAL_MODES]
                             [2];    // fixed joystick gains for each mode
                                     // specified in the config file
  Eigen::Matrix<double, 3, 1> com_;  // Current center of mass
  double joystick_forw,
      joystick_spin;  // forw and spin motion control references
  struct timespec t_now_, t_prev_;
  double dt_;
  double u_theta_, u_x_,
      u_spin_;  // individual components of the wheel current

  Krang::Hardware*
      krang_;  // interface to hardware components (sensors and motors)
  dart::dynamics::SkeletonPtr robot_;  // dart object with krang's skeleton

  bool dynamic_lqr_;  // if true, online pose-dependent lqr gains will be used
                      // instead of the fixed gains specified in the config
                      // file
  Eigen::Matrix<double, 4, 4> lqrQ_;  // Q matrix for LQR
  Eigen::Matrix<double, 1, 1> lqrR_;  // R matrix for LQR

  double to_bal_threshold_;  // if CoM angle error < value, STAND mode
                             // automatically transitions to BAL_LO
  double imu_sit_angle_;     // if angle < value, SIT mode automatically
                             // transitions to GROUND_LO mode

  Mpc mpc_;
  BalanceMode previous_balance_mode_;
};
#endif  // KRANG_BALANCING_CONTROL_H_
