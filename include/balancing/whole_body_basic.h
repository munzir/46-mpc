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
 * @file whole_body_basic.h
 * @author Munzir Zafar
 * @date Apr 17, 2019
 * @brief Tracks pre-computed arm trajectories for tray-carrying
 */

#ifndef KRANG_SIMULATION_WHOLE_BODY_BASIC_H_
#define KRANG_SIMULATION_WHOLE_BODY_BASIC_H_

#include <Eigen/Eigen>  // Eigen::MatrixXd
#include <fstream>     // std::ofstream
#include <string>       // std::string

class WholeBodyBasic {
 public:
  WholeBodyBasic(const std::string& path_to_arm_trajectories);
  ~WholeBodyBasic() { q_log_.close(); };
  Eigen::VectorXd GetPositionTrajRef(const Eigen::MatrixXd& q_traj,
                                     const double time);
  Eigen::VectorXd GetSpeedTrajRef(const Eigen::MatrixXd& dq_traj,
                                  const double time);
  void ComputeReferenceSpeeds(const double time, const double* q_left,
                              const double* q_right, double* dqref_left,
                              double* dqref_right);
  void Log(const double time, const double* q_left, const double* dq_left,
           const double* q_right, const double* dq_right);
  void SetInitTime(const double time);
  double GetInitTime();
  const int num_joints_;
  Eigen::MatrixXd q_left_traj_, dq_left_traj_, q_right_traj_, dq_right_traj_;
  Eigen::MatrixXd gains_left_, gains_right_;
  double init_time_;
  std::ofstream q_log_;
};

#endif  // KRANG_SIMULATION_WHOLE_BODY_BASIC_H_
