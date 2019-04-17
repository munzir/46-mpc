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
 * @file whole_body_basic.cpp
 * @author Munzir Zafar
 * @date Apr 17, 2019
 * @brief Tracks pre-computed arm trajectories for tray-carrying
 */

#include "balancing/whole_body_basic.h"
#include <config4cpp/Configuration.h>
#include <math.h>                    // round()
#include <krang-utils/file_ops.hpp>  // readInputFileAsMatrix()
#include <sstream>                   // std::stringstream
#include <string>                    // std::string

WholeBodyBasic::WholeBodyBasic(const std::string& path_to_arm_trajectories)
    : num_joints_(7) {
  bool verbose = false;
  try {
    std::cout << "Reading arm trajectories ...\n";
    q_left_traj_ = readInputFileAsMatrix(
        path_to_arm_trajectories + "q_left_traj", verbose);
    std::cout << "q_left_traj_ size: " << q_left_traj_.rows() << ", "
              << q_left_traj_.cols() << std::endl;
    dq_left_traj_ = readInputFileAsMatrix(
        path_to_arm_trajectories + "dq_left_traj", verbose);
    std::cout << "dq_left_traj_ size: " << q_left_traj_.rows() << ", "
              << q_left_traj_.cols() << std::endl;
    q_right_traj_ = readInputFileAsMatrix(
        path_to_arm_trajectories + "q_right_traj", verbose);
    std::cout << "q_right_traj_ size: " << q_right_traj_.rows() << ", "
              << q_right_traj_.cols() << std::endl;
    dq_right_traj_ = readInputFileAsMatrix(
        path_to_arm_trajectories + "dq_right_traj", verbose);
    std::cout << "dq_right_traj_ size: " << dq_right_traj_.rows() << ", "
              << dq_right_traj_.cols() << std::endl;
    std::cout << "|-> Done\n";
  } catch (exception& e) {
    std::cout << e.what() << std::endl;
    assert(false && "Problem loading arm trajectories");
  }

  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << std::endl
            << "Reading whole body basic parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse("/usr/local/share/krang/balancing/cfg/whole_body_basic.cfg");

    // Gains for left arm
    gains_left_ = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
    str = cfg->lookupString(scope, "gains_left");
    stream.str(str);
    for (int j = 0; j < num_joints_; j++) stream >> gains_left_(j, j);
    stream.clear();
    std::cout << "gains_left: " << gains_left_.diagonal().transpose()
              << std::endl;

    // Gains for right arm
    gains_right_ = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
    str = cfg->lookupString(scope, "gains_right");
    stream.str(str);
    for (int j = 0; j < num_joints_; j++) stream >> gains_right_(j, j);
    stream.clear();
    std::cout << "gains_right: " << gains_right_.diagonal().transpose()
              << std::endl;

  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading whole body basic parameters");
  }
  q_log_ = std::ofstream("/var/tmp/krangmpc/q_log");
}

Eigen::VectorXd WholeBodyBasic::GetPositionTrajRef(
    const Eigen::MatrixXd& q_traj, const double time) {
  const double dt = 0.001;
  const int index = round(time / dt);
  if (index < q_traj.rows())
    return q_traj.row(index).transpose();
  else
    return q_traj.bottomRows(1).transpose();
}

Eigen::VectorXd WholeBodyBasic::GetSpeedTrajRef(const Eigen::MatrixXd& dq_traj,
                                                const double time) {
  const double dt = 0.001;
  const int index = round(time / dt);
  if (index < dq_traj.rows())
    return dq_traj.row(index).transpose();
  else
    return Eigen::VectorXd::Zero(num_joints_);
}

void WholeBodyBasic::SetInitTime(const double time) { init_time_ = time; }

double WholeBodyBasic::GetInitTime() { return init_time_; }

void WholeBodyBasic::ComputeReferenceSpeeds(const double time,
                                            const double* q_left,
                                            const double* q_right,
                                            double* dqref_left,
                                            double* dqref_right) {
  // -- Compute left arm reference speeds
  // convert from double array to Eigen vector
  Eigen::VectorXd q_left_eig = Eigen::VectorXd::Zero(num_joints_);
  for (int i = 0; i < num_joints_; i++) q_left_eig(i) = q_left[i];

  // dq_reference = dqref - Kp * (q - qref)
  Eigen::VectorXd dqref_left_eig =
      GetSpeedTrajRef(dq_left_traj_, time) -
      gains_left_ * (q_left_eig - GetPositionTrajRef(q_left_traj_, time));

  // convert from Eigen vector to double array
  for (int i = 0; i < num_joints_; i++) dqref_left[i] = dqref_left_eig(i);

  // -- Compute right arm reference speeds
  // convert from double array to Eigen vector
  Eigen::VectorXd q_right_eig = Eigen::VectorXd::Zero(num_joints_);
  for (int i = 0; i < num_joints_; i++) q_right_eig(i) = q_right[i];

  // dq_reference = dqref - Kp * (q - qref)
  Eigen::VectorXd dqref_right_eig =
      GetSpeedTrajRef(dq_right_traj_, time) -
      gains_right_ * (q_right_eig - GetPositionTrajRef(q_right_traj_, time));

  // convert from Eigen vector to double array
  for (int i = 0; i < num_joints_; i++) dqref_right[i] = dqref_right_eig(i);
}

void WholeBodyBasic::Log(const double time, const double* q_left,
                         const double* dq_left, const double* q_right,
                         const double* dq_right) {
  // time
  q_log_ << time;

  // qref_left
  q_log_ << " " << GetPositionTrajRef(q_left_traj_, time).transpose();

  // q_left
  for (int i = 0; i < num_joints_; i++) q_log_ << " " << q_left[i];

  // dqref_left
  q_log_ << " " << GetSpeedTrajRef(dq_left_traj_, time).transpose();

  // dq_left
  for (int i = 0; i < num_joints_; i++) q_log_ << " " << dq_left[i];

  // qref_right
  q_log_ << " " << GetPositionTrajRef(q_right_traj_, time).transpose();

  // q_right
  for (int i = 0; i < num_joints_; i++) q_log_ << " " << q_right[i];

  // dqref_right
  q_log_ << " " << GetSpeedTrajRef(dq_right_traj_, time).transpose();

  // dq_right
  for (int i = 0; i < num_joints_; i++) q_log_ << " " << dq_right[i];

  q_log_ << std::endl;
}
