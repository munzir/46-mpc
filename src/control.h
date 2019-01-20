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
 * @date Oct 31, 2018
 * @brief Header file for control.cpp that implements balancing control
 * functions
 */

#ifndef KRANG_BALANCING_CONTROL_H_
#define KRANG_BALANCING_CONTROL_H_

#include <Eigen/Eigen>    // Eigen::MatrixXd, Eigen::Matrix<double, #, #>
#include <dart/dart.hpp>  // dart::dynamics::SkeletonPtr
#include <kore.hpp>       // Krang::Hardware

#include "balancing_config.h"  // BalancingConfig

class BalanceControl {
 public:
  BalanceControl(Krang::Hardware* krang_, dart::dynamics::SkeletonPtr robot_,
                 BalancingConfig& params);
  ~BalanceControl() {}
  enum BalanceMode {
    GROUND_LO = 0,
    STAND,
    SIT,
    BAL_LO,
    BAL_HI,
    GROUND_HI,
    NUM_MODES
  };
  static const char MODE_STRINGS[][16];
  double ElapsedTimeSinceLastCall();
  void UpdateState();
  void CancelPositionBuiltup();
  void ForceModeChange(BalanceMode new_mode);
  void BalancingController(double* control_input);
  void ChangePdGain(int index, double change);
  void Print();
  void StandSitEvent();
  void BalHiLoEvent();
  void SetFwdInput(double forw);
  void SetSpinInput(double spin);

 private:
  void SetComParameters(Eigen::MatrixXd beta_params, int num_body_params);
  void UpdateReference(const double& forw, const double& spin);
  void ComputeCurrent(const Eigen::Matrix<double, 6, 1>& pd_gain,
                      const Eigen::Matrix<double, 6, 1>& error,
                      double* control_input);
  Eigen::MatrixXd ComputeLqrGains();

 private:
  BalanceMode balance_mode_;
  Eigen::Matrix<double, 4, 4> lqr_hack_ratios_;
  Eigen::Matrix<double, 6, 1> pd_gains_, ref_state_, state_, error_;
  Eigen::Matrix<double, 6, 1> pd_gains_list_[NUM_MODES];
  double joystick_gains_list_[NUM_MODES][2];
  Eigen::Matrix<double, 3, 1> com_;
  double joystick_forw, joystick_spin;
  struct timespec t_now_, t_prev_;
  double dt_;
  double u_theta_, u_x_, u_spin_;

  Krang::Hardware* krang_;
  dart::dynamics::SkeletonPtr robot_;

  bool dynamic_lqr_;
  Eigen::Matrix<double, 4, 4> lqrQ_;
  Eigen::Matrix<double, 1, 1> lqrR_;

  double to_bal_threshold_, imu_sit_angle_;
};
#endif  // KRANG_BALANCING_CONTROL_H_
