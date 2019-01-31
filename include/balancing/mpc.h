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
 * @file mpc.h
 * @author Munzir Zafar
 * @date Jan 27, 2018
 * @brief Header for mpc.cpp that contains code for doing mpc in a separate
 * thread that works in conjunction with the main thread of the balancing code
 */

#ifndef KRANG_BALANCING_MPC_H_
#define KRANG_BALANCING_MPC_H_

#include <Eigen/Eigen>         // Eigen::MatrixXd, Eigen::Matrix<double, #, #>
#include <condition_variable>  // std::condition_variable
#include <dart/dart.hpp>       // dart::dynamics::SkeletonPtr
#include <mutex>               // std::mutex
#include <thread>              // std::thread

#include "balancing/ddp_objects.h"  // TwipDynamics...

class Mpc {
 public:
  Mpc(const char* mpc_config_file);
  ~Mpc() { Destroy(); }
  void Destroy();

  enum DdpMode {
    DDP_IDLE = 0,
    DDP_COMPUTE_TRAJ,
    DDP_TRAJ_OK,
    DDP_FOR_MPC,
    NUM_DDP_MODES
  };
  static const char DDP_MODE_STRINGS[][32];

  struct ConfigParameters {
    // Path to urdf file
    char three_dof_urdf_path_[1024];

    // DDP Parameters
    struct DdpParameters {
      double final_time_;
      Eigen::Matrix<double, 8, 1> goal_state_;
      int max_iter_;
      TwipDynamicsCost<double>::StateHessian state_hessian_;
      TwipDynamicsCost<double>::ControlHessian control_hessian_;
      TwipDynamicsTerminalCost<double>::Hessian terminal_state_hessian_;
    } ddp_;

    // Output file for saving initial trajectory
    char initial_trajectory_output_path_[1024];

    // MPC Parameters
    struct MpcParameters {
      int max_iter_;
      int horizon_;
      TwipDynamicsCost<double>::StateHessian state_hessian_;
      TwipDynamicsCost<double>::ControlHessian control_hessian_;
      TwipDynamicsTerminalCost<double>::Hessian terminal_state_hessian_;
      double dt_;
    } mpc_;
  };

  // DDP Mode get and set. Mutex-based access.
  DdpMode GetDdpMode();
  void SetDdpMode(DdpMode ddp_mode);

  // DDP Thread function
  void DdpThread();

  // Set current time as init_time_
  void SetInitTime();

  // Get init_time_
  double GetInitTime();

 private:
  // Read configuration parameters
  void ReadConfigParameters(const char* mpc_config_file);

  // Update Three Dof Robot
  void UpdateThreeDof(dart::dynamics::SkeletonPtr& robot,
                      dart::dynamics::SkeletonPtr& three_dof_robot);

  // Get dynamics for ddp from 3-dof dart skeleton
  void DartSkeletonToTwipDynamics(dart::dynamics::SkeletonPtr& three_dof_robot,
                                  TwipDynamics<double>* twip_dynamics);

 public:
  ConfigParameters param_;
  DdpMode ddp_mode_;  // Current mode of the ddp thread state machine
  std::mutex ddp_mode_mutex_;
  std::condition_variable ddp_mode_change_signal_;
  std::thread* ddp_thread_;
  bool ddp_thread_run_;
  std::mutex ddp_thread_run_mutex_;
  dart::dynamics::SkeletonPtr
      ddp_robot_;  // A clone of the Skeleton in the main balancing thread so
                   // that we don't have to share the entire Skeleton using
                   // mutex
  Eigen::VectorXd robot_pose_;
  std::mutex robot_pose_mutex_;
  dart::dynamics::SkeletonPtr three_dof_robot_;
  std::mutex three_dof_robot_mutex_;
  struct Heading {
    double distance_;
    double direction_;
  } ddp_init_heading_;
  std::mutex ddp_init_heading_mutex_;
  struct AugmentedState {
    double x0_;
    double y0_;
  } ddp_augmented_state_;
  std::mutex ddp_augmented_state_mutex_;
  Eigen::Matrix<double, 6, 1>
      ddp_bal_state_;  // copy of main thread's state variable to be
                       // mutex-shared by ddp thread
  std::mutex ddp_bal_state_mutex_;
  struct DdpTrajectory {
    TwipDynamics<double>::StateTrajectory state_;
    TwipDynamics<double>::ControlTrajectory control_;
  } ddp_trajectory_;
  bool done_;  // When mpc has finished following the trajectory
  std::mutex done_mutex_;
  double init_time_;
  std::mutex init_time_mutex_;
  TwipDynamics<double>::ControlTrajectory mpc_trajectory_main_,
      mpc_trajectory_backup_;
  std::mutex mpc_trajectory_main_mutex_, mpc_trajectory_backup_mutex_;
};

#endif  // KRANG_BALANCING_MPC_H_
