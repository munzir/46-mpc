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

#include "balancing/ddp_objects.h"  // TwipDynamicsExt..., CsvWriter

class Mpc {
 public:
  enum DdpMode {
    DDP_IDLE = 0,
    DDP_COMPUTE_TRAJ,
    DDP_TRAJ_OK,
    DDP_FOR_MPC,
    NUM_DDP_MODES
  };

  struct ConfigParameters {
    // Path to urdf file
    char three_dof_urdf_path_[1024];

    // DDP Parameters
    struct DdpParameters {
      double final_time_;
      TwipDynamicsExtCost<double>::State goal_state_;
      int max_iter_;
      TwipDynamicsExtCost<double>::StateHessian state_hessian_;
      TwipDynamicsExtCost<double>::ControlHessian control_hessian_;
      TwipDynamicsExtTerminalCost<double>::Hessian terminal_state_hessian_;
    } ddp_;

    // Output file for saving initial trajectory
    char initial_trajectory_output_path_[1024];

    // MPC Parameters
    struct MpcParameters {
      int max_iter_;
      int horizon_;
      TwipDynamicsExtCost<double>::StateHessian state_hessian_;
      TwipDynamicsExtCost<double>::ControlHessian control_hessian_;
      TwipDynamicsExtTerminalCost<double>::Hessian terminal_state_hessian_;
      double dt_;
    } mpc_;

    // Output file for saving final trajectory
    char final_trajectory_output_path_[1024];

    // Exit thresholds
    struct ExitThreshold {
      double dx_;
      double dpsi_;
      double dtheta_;
    } exit_threshold_;
  };

  Mpc(const char* mpc_config_file);
  ~Mpc() { Destroy(); }

  // DDP Mode get and set. Mutex-based access.
  DdpMode GetDdpMode();
  void SetDdpMode(DdpMode ddp_mode);

  // Prepare to enter Mpc mode
  void InitializeMpcObjects();

  // MPC control. This is to be used by the main thread in mpc mode
  void Control(double* control_input);

  // time setting and getting
  void SetTime(double time);
  double GetTime();

  // For printing the current mode
  static const char DDP_MODE_STRINGS[][32];

  // A clone of the Skeleton in the main balancing thread so that we don't have
  // to share the entire Skeleton using mutex
  dart::dynamics::SkeletonPtr robot_;

  // Max current that can be supplied to the wheel. Set in BalanceControl
  // constructor
  double max_input_current_;

  // Shared variables to update the dynamics and state of the 3-dof robot in the
  // ddp thread
  // TODO: Could be done via setters and getters perhaps
  Eigen::VectorXd robot_pose_;
  std::mutex robot_pose_mutex_;
  Eigen::Matrix<double, 6, 1> state_;
  std::mutex state_mutex_;
  struct AugmentedState {
    double x0_;
    double y0_;
  } augmented_state_;
  std::mutex augmented_state_mutex_;
  struct Heading {
    double distance_;
    double direction_;
    double tilt_;
  } init_heading_;
  std::mutex init_heading_mutex_;
  double time_;
  std::mutex time_mutex_;

  // If mpc is done, this variable will let the main thread know
  bool done_;

  // If all the speeds are under threshold
  bool static_;

 private:
  // Destroy function called by the destructor
  void Destroy();

  // DDP Thread function
  void DdpThread();

  // Get init_time_
  double GetInitTime();

  // Set current time as init_time_
  void SetInitTime();

  // Get dynamics for ddp from 3-dof dart skeleton
  void DartSkeletonToTwipDynamics(dart::dynamics::SkeletonPtr& three_dof_robot,
                                  TwipDynamics<double>* twip_dynamics);

  // Read configuration parameters
  void ReadConfigParameters(const char* mpc_config_file);

  // Update Three Dof Robot
  void UpdateThreeDof(dart::dynamics::SkeletonPtr& robot,
                      dart::dynamics::SkeletonPtr& three_dof_robot);

  ConfigParameters param_;
  DdpMode mode_;  // Current mode of the ddp thread state machine
  std::mutex mode_mutex_;
  std::condition_variable mode_change_signal_;
  std::thread* thread_;
  bool thread_run_;
  std::mutex thread_run_mutex_;
  dart::dynamics::SkeletonPtr three_dof_robot_;
  std::mutex three_dof_robot_mutex_;
  struct DdpTrajectory {
    TwipDynamics<double>::StateTrajectory state_;
    TwipDynamics<double>::ControlTrajectory control_;
  } ddp_trajectory_;
  double init_time_;
  std::mutex init_time_mutex_;
  TwipDynamics<double>::ControlTrajectory mpc_trajectory_main_,
      mpc_trajectory_backup_;
  Eigen::Matrix<int, Eigen::Dynamic, 1> updating_iteration_main_,
      updating_iteration_backup_;  // Keep track of which iteration number of
                                   // mpc updated each entry of the control
                                   // trajectories. Begins with -1.0 i.e. after
                                   // DDP_COMPUTE_TRAJ computes the full
                                   // reference trajectory
  std::mutex mpc_trajectory_main_mutex_, mpc_trajectory_backup_mutex_;
  CsvWriter<double> writer_;
  TwipDynamics<double>::Control current_u_;
  std::mutex current_u_mutex_;
};

#endif  // KRANG_BALANCING_MPC_H_
