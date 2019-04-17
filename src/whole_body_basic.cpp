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
#include <math.h>                    // round()
#include <krang-utils/file_ops.hpp>  // readInputFileAsMatrix()
#include <string>                    // std::string

WholeBodyBasic::WholeBodyBasic(const std::string& path_to_arm_trajectories) {
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
}

Eigen::VectorXd WholeBodyBasic::GetCurrentRef(const Eigen::MatrixXd& q_traj,
                                              const double time) {
  const double dt = 0.001;
  const int index = round(time / dt);
  return q_traj.row(index);
}

