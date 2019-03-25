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
 * @file mpc.cpp
 * @author Munzir Zafar
 * @date Jan 27, 2019
 * @brief Code for doing mpc in a separate thread that works in
 * conjunction with the main thread of the balancing code
 */

#include "balancing/mpc.h"

#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <Eigen/Eigen>    // Eigen::MatrixXd, Eigen::Matrix<double, #, #>
#include <algorithm>      // std::max(), std::min()
#include <dart/dart.hpp>  // dart::dynamics::SkeletonPtr
#include <dart/utils/urdf/urdf.hpp>  // dart::utils::DartLoader
#include <ddp/ddp.hpp>               // optimizer, OptimizerResult
#include <ddp/util.hpp>              // util::DefaultLogger, util::time_steps
#include <mutex>                     // std::mutex
#include <sstream>                   // std::stringstream
#include <thread>                    // std::thread

#include "balancing/ddp_objects.h"  // TwipDynamics...

//============================================================================
const char Mpc::DDP_MODE_STRINGS[][32] = {"DDP Idle", "DDP Compute Traj",
                                          "DDP Traj OK?", "DDP for MPC"};

//============================================================================
Mpc::Mpc(const char* mpc_config_file) {
  ReadConfigParameters(mpc_config_file);
  writer_.OpenFile(param_.final_trajectory_output_path_);

  // Launch the DDP thread
  dart::utils::DartLoader loader;
  three_dof_robot_ = loader.parseSkeleton(param_.three_dof_urdf_path_);
  three_dof_robot_->getJoint(0)->setDampingCoefficient(0, 0.5);
  three_dof_robot_->getJoint(1)->setDampingCoefficient(0, 0.5);
  thread_run_ = true;
  SetDdpMode(DDP_IDLE);
  thread_ = new std::thread(&Mpc::DdpThread, this);
}

//============================================================================
void Mpc::ReadConfigParameters(const char* mpc_config_file) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << std::endl
            << "Reading MPC configuration parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(mpc_config_file);

    // Read the path to 3-dof urdf file
    strcpy(param_.three_dof_urdf_path_,
           cfg->lookupString(scope, "three_dof_urdf_path"));
    std::cout << "three_dof_urdf_path: " << param_.three_dof_urdf_path_
              << std::endl;

    // =======================
    // DDP Parameters
    param_.ddp_.final_time_ = cfg->lookupFloat(scope, "ddp_final_time");
    std::cout << "ddp_final_time:" << param_.ddp_.final_time_ << std::endl;

    str = cfg->lookupString(scope, "ddp_goal_state");
    stream.str(str);
    for (int i = 0; i < 8; i++) stream >> param_.ddp_.goal_state_(i);
    stream.clear();
    std::cout << "ddp_goal_state: " << param_.ddp_.goal_state_.transpose()
              << std::endl;

    param_.ddp_.max_iter_ = cfg->lookupInt(scope, "ddp_max_iter");
    std::cout << "ddp_max_iter:" << param_.ddp_.max_iter_ << std::endl;

    param_.ddp_.state_hessian_.setZero();
    str = cfg->lookupString(scope, "ddp_state_penalties");
    stream.str(str);
    for (int i = 0; i < 8; i++) stream >> param_.ddp_.state_hessian_(i, i);
    stream.clear();
    std::cout << "ddp_state_penalties: "
              << param_.ddp_.state_hessian_.diagonal().transpose() << std::endl;

    param_.ddp_.control_hessian_.setZero();
    str = cfg->lookupString(scope, "ddp_control_penalties");
    stream.str(str);
    for (int i = 0; i < 2; i++) stream >> param_.ddp_.control_hessian_(i, i);
    stream.clear();
    std::cout << "ddp_control_penalties: "
              << param_.ddp_.control_hessian_.diagonal().transpose()
              << std::endl;

    param_.ddp_.terminal_state_hessian_.setZero();
    str = cfg->lookupString(scope, "ddp_terminal_state_penalties");
    stream.str(str);
    for (int i = 0; i < 8; i++)
      stream >> param_.ddp_.terminal_state_hessian_(i, i);
    stream.clear();
    std::cout << "ddp_terminal_state_penalties: "
              << param_.ddp_.terminal_state_hessian_.diagonal().transpose()
              << std::endl;

    // Read the path to save initial trajector
    strcpy(param_.initial_trajectory_output_path_,
           cfg->lookupString(scope, "initial_trajectory_output_path"));
    std::cout << "initial_trajectory_output_path: "
              << param_.initial_trajectory_output_path_ << std::endl;

    // =======================
    // MPC Parameters
    param_.mpc_.max_iter_ = cfg->lookupInt(scope, "mpc_max_iter");
    std::cout << "mpc_max_iter:" << param_.mpc_.max_iter_ << std::endl;

    param_.mpc_.horizon_ = cfg->lookupInt(scope, "mpc_horizon");
    std::cout << "mpc_horizon:" << param_.mpc_.horizon_ << std::endl;

    param_.mpc_.state_hessian_.setZero();
    str = cfg->lookupString(scope, "mpc_state_penalties");
    stream.str(str);
    for (int i = 0; i < 8; i++) stream >> param_.mpc_.state_hessian_(i, i);
    stream.clear();
    std::cout << "mpc_state_penalties: "
              << param_.mpc_.state_hessian_.diagonal().transpose() << std::endl;

    param_.mpc_.control_hessian_.setZero();
    str = cfg->lookupString(scope, "mpc_control_penalties");
    stream.str(str);
    for (int i = 0; i < 2; i++) stream >> param_.mpc_.control_hessian_(i, i);
    stream.clear();
    std::cout << "mpc_control_penalties: "
              << param_.mpc_.control_hessian_.diagonal().transpose()
              << std::endl;

    param_.mpc_.terminal_state_hessian_.setZero();
    str = cfg->lookupString(scope, "mpc_terminal_state_penalties");
    stream.str(str);
    for (int i = 0; i < 8; i++)
      stream >> param_.mpc_.terminal_state_hessian_(i, i);
    stream.clear();
    std::cout << "mpc_terminal_state_penalties: "
              << param_.mpc_.terminal_state_hessian_.diagonal().transpose()
              << std::endl;

    param_.mpc_.dt_ = cfg->lookupFloat(scope, "mpc_dt");
    std::cout << "mpc_dt:" << param_.mpc_.dt_ << std::endl;

    // Read the path to save final trajectory
    strcpy(param_.final_trajectory_output_path_,
           cfg->lookupString(scope, "final_trajectory_output_path"));
    std::cout << "final_trajectory_output_path: "
              << param_.final_trajectory_output_path_ << std::endl;

    // Exit thresholds
    param_.exit_threshold_.dx_ = cfg->lookupFloat(scope, "exit_threshold_dx");
    std::cout << "exit_threshold_dx:" << param_.exit_threshold_.dx_
              << std::endl;
    param_.exit_threshold_.dpsi_ =
        cfg->lookupFloat(scope, "exit_threshold_dpsi");
    std::cout << "exit_threshold_dpsi:" << param_.exit_threshold_.dpsi_
              << std::endl;
    param_.exit_threshold_.dtheta_ =
        cfg->lookupFloat(scope, "exit_threshold_dtheta");
    std::cout << "exit_threshold_dtheta:" << param_.exit_threshold_.dtheta_
              << std::endl;

  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading mpc config parameters");
  }
  std::cout << std::endl;
}

//============================================================================
Mpc::DdpMode Mpc::GetDdpMode() {
  mode_mutex_.lock();
  DdpMode mode = mode_;
  mode_mutex_.unlock();
  return mode;
}

//============================================================================
void Mpc::SetDdpMode(DdpMode ddp_mode) {
  mode_mutex_.lock();
  DdpMode last_mode = mode_;
  mode_ = ddp_mode;
  mode_mutex_.unlock();
  if (last_mode != mode_) mode_change_signal_.notify_all();
}

//============================================================================
// Update the position and velocity of a 3dof dart model with that of the global
// krang model
void Mpc::UpdateThreeDof(dart::dynamics::SkeletonPtr& robot,
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
void Mpc::DartSkeletonToTwipDynamics(
    dart::dynamics::SkeletonPtr& three_dof_robot,
    TwipDynamics<double>* twip_dynamics) {
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

  // Set the parameters for twip dynamics object
  twip_dynamics->SetParameters(p);
}

//============================================================================
void Mpc::SetInitTime() {
  init_time_mutex_.lock();
  init_time_ = GetTime();
  init_time_mutex_.unlock();
}

//============================================================================
// return ddp init time
double Mpc::GetInitTime() {
  double temp;
  init_time_mutex_.lock();
  temp = init_time_;
  init_time_mutex_.unlock();
  return temp;
}

//============================================================================
void Mpc::DdpThread() {
  std::cout << "Entering DDP Thread ..." << std::endl;

  int current_step;
  bool run = true;
  while (run) {
    // Implementation for each DDP mode
    switch (GetDdpMode()) {
      case DDP_IDLE: {
        break;
      }
      case DDP_COMPUTE_TRAJ: {
        ////// Time steps
        int time_steps =
            util::time_steps(param_.ddp_.final_time_, param_.mpc_.dt_);

        ////// Initial State
        // Lock mutexes
        state_mutex_.lock();
        init_heading_mutex_.lock();
        augmented_state_mutex_.lock();

        // Define initial heading for computation of future mpc states
        init_heading_.distance_ = 0.25 * state_(2);
        init_heading_.direction_ = 0.25 / 0.68 * 2.0 * state_(4);
        init_heading_.tilt_ = state_(0);

        // Reset augmented state
        augmented_state_.x0_ = 0.0;
        augmented_state_.y0_ = 0.0;

        // Initial state for ddp trajectory computation
        TwipDynamics<double>::State x0;
        x0 << 0.25 * state_(2) - init_heading_.distance_,
            0.25 / 0.68 * 2.0 * state_(4) - init_heading_.direction_,
            state_(0) - init_heading_.tilt_, 0.0, 0.0, 0.0, 0.0, 0.0;
        // 0.25 * state_(3),
        // 0.25 / 0.68 * 2.0 * state_(5), state_(1), augmented_state_.x0_,
        // augmented_state_.y0_;

        // Unlock mutexes
        augmented_state_mutex_.unlock();
        init_heading_mutex_.unlock();
        state_mutex_.unlock();

        ////// Dynamics
        robot_pose_mutex_.lock();
        robot_->setPositions(robot_pose_);
        robot_pose_mutex_.unlock();
        UpdateThreeDof(robot_, three_dof_robot_);
        TwipDynamics<double> ddp_dynamics;
        DartSkeletonToTwipDynamics(three_dof_robot_, &ddp_dynamics);

        ////// Costs
        TwipDynamicsCost<double> ddp_cost(param_.ddp_.goal_state_,
                                          param_.ddp_.state_hessian_,
                                          param_.ddp_.control_hessian_);
        TwipDynamicsTerminalCost<double> ddp_terminal_cost(
            param_.ddp_.goal_state_, param_.ddp_.terminal_state_hessian_);

        ////// Nominal trajectory
        auto nominal_traj =
            TwipDynamics<double>::ControlTrajectory::Zero(2, time_steps);

        ////// Perform optimization
        util::DefaultLogger logger;
        bool verbose = true;
        optimizer::DDP<TwipDynamics<double>> ddp_optimizer(
            param_.mpc_.dt_, time_steps, param_.mpc_.max_iter_, &logger,
            verbose);
        OptimizerResult<TwipDynamics<double>> optimizer_result =
            ddp_optimizer.run(x0, nominal_traj, ddp_dynamics, ddp_cost,
                              ddp_terminal_cost);

        ////// Upon failure, recompute
        if (!optimizer_result.success) {
          std::cout << std::endl
                    << "[ERR] DDP computation failed, trying again"
                    << std::endl;
          break;
        }

        ////// Save the trajectory
        ddp_trajectory_.state_ = optimizer_result.state_trajectory;
        ddp_trajectory_.control_ = optimizer_result.control_trajectory;
        CsvWriter<double> writer;
        writer.SaveTrajectory(ddp_trajectory_.state_, ddp_trajectory_.control_,
                              param_.initial_trajectory_output_path_);

        ////// Change the mode to DDP_TRAJ_OK that waits for the user to give
        ////// a green signal. But before that, see if we are still in the
        ////// current mode, lest an external event has forced us out
        if (GetDdpMode() == DDP_COMPUTE_TRAJ) SetDdpMode(DDP_TRAJ_OK);

        break;
      }
      case DDP_TRAJ_OK: {
        // Display the plot
        std::stringstream python_plot_command;
        python_plot_command
            << "python3 /usr/local/bin/krang/mpc/plot_script.py "
            << param_.initial_trajectory_output_path_ << " &";
        int e = system(python_plot_command.str().c_str());

        // current_step was originally local to DDP_FOR_MPC mode
        // but when running simulation with very small dt, this could
        // result in recomputation of ddp multiple times in a single time step.
        // To prevent recomputation, we are now keeping track of changes to
        // current_step, hence a need to remember current_step across iterations
        // in order to compare with value in last iteration so that computation
        // is performed only when current_step has incremented. In order for
        // this check to be true in the first iteration, current_step needs to
        // be -1
        current_step = -1;

        // Stay here until an external event changes the mode_
        std::unique_lock<std::mutex> lock(mode_mutex_);
        while (mode_ == DDP_TRAJ_OK) mode_change_signal_.wait(lock);

        break;
      }
      case DDP_FOR_MPC: {
        // Current step
        double time_now = GetTime();
        double init_time = GetInitTime();
        int current_step_in_last_iteration = current_step;
        current_step = floor((time_now - init_time) / param_.mpc_.dt_);

        // Don't perform DDP again if we are still on the same step
        if (current_step <= current_step_in_last_iteration) break;

        // Horizon
        int trajectory_size = ddp_trajectory_.state_.cols() - 1;
        int horizon = (current_step + param_.mpc_.horizon_ < trajectory_size)
                          ? param_.mpc_.horizon_
                          : trajectory_size - current_step;
        // Initial State
        state_mutex_.lock();
        init_heading_mutex_.lock();
        augmented_state_mutex_.lock();
        TwipDynamics<double>::State x0;
        x0 << 0.25 * state_(2) - init_heading_.distance_,
            0.25 / 0.68 * 2.0 * state_(4) - init_heading_.direction_,
            state_(0) - init_heading_.tilt_, 0.25 * state_(3),
            0.25 / 0.68 * 2.0 * state_(5), state_(1), augmented_state_.x0_,
            augmented_state_.y0_;
        augmented_state_mutex_.unlock();
        init_heading_mutex_.unlock();
        state_mutex_.unlock();

        // Dynamics
        robot_pose_mutex_.lock();
        robot_->setPositions(robot_pose_);
        robot_pose_mutex_.unlock();
        UpdateThreeDof(robot_, three_dof_robot_);
        TwipDynamics<double> ddp_dynamics;
        DartSkeletonToTwipDynamics(three_dof_robot_, &ddp_dynamics);

        // Costs
        TwipDynamics<double>::State terminal_state =
            ddp_trajectory_.state_.col(current_step + horizon);
        TwipDynamicsCost<double> ddp_cost(terminal_state,
                                          param_.mpc_.state_hessian_,
                                          param_.mpc_.control_hessian_);
        TwipDynamicsTerminalCost<double> ddp_terminal_cost(
            terminal_state, param_.mpc_.terminal_state_hessian_);

        // Nominal trajectory
        // TODO: Warm start
        auto nominal_traj =
            TwipDynamics<double>::ControlTrajectory::Zero(2, horizon);
        OptimizerResult<TwipDynamics<double>> optimizer_result;
        optimizer_result.control_trajectory =
            nominal_traj;  // I don't see why this is needed. For now just
                           // copying Shimin's logic

        // Reference trajectory
        TwipDynamics<double>::StateTrajectory reference_traj =
            ddp_trajectory_.state_.block(0, current_step, 8, horizon);

        // Perform optimization
        util::DefaultLogger logger;
        bool verbose = true;
        optimizer::DDP<TwipDynamics<double>> ddp_optimizer(
            param_.mpc_.dt_, horizon, param_.mpc_.max_iter_, &logger, verbose);
        optimizer_result = ddp_optimizer.run_horizon(
            x0, nominal_traj, reference_traj, ddp_dynamics, ddp_cost,
            ddp_terminal_cost);

        // Upon failure, keep moving without updating the trajectory
        // TODO: Maybe we should exit mpc mode in the main thread
        if (!optimizer_result.success) {
          std::cout << std::endl << "[ERR] DDP for MPC failed" << std::endl;
          break;
        }

        // Save the trajectory
        mpc_trajectory_main_mutex_.lock();
        mpc_trajectory_main_.block(0, current_step, 2, horizon) =
            optimizer_result.control_trajectory;
        updating_iteration_main_.segment(current_step, horizon) =
            current_step * Eigen::Matrix<int, Eigen::Dynamic, 1>::Ones(horizon);
        mpc_trajectory_main_mutex_.unlock();
        mpc_trajectory_backup_mutex_.lock();
        mpc_trajectory_backup_.block(0, current_step, 2, horizon) =
            optimizer_result.control_trajectory;
        updating_iteration_backup_.segment(current_step, horizon) =
            current_step * Eigen::Matrix<int, Eigen::Dynamic, 1>::Ones(horizon);
        mpc_trajectory_backup_mutex_.unlock();

        break;
      }
    }

    // Loop will exit if thread_run_ is reset
    thread_run_mutex_.lock();
    run = thread_run_;
    thread_run_mutex_.unlock();
  }
  std::cout << "Exiting DDP Thread ..." << std::endl;
}

//============================================================================
void Mpc::InitializeMpcObjects() {
  mpc_trajectory_main_ = ddp_trajectory_.control_;
  mpc_trajectory_backup_ = ddp_trajectory_.control_;

  // Updating iteration number defaults to -1 indicating that it is not yet
  // updated by the mpc iterations but only by the DDP_COMPUTE_TRAJ
  int traj_size = ddp_trajectory_.control_.cols();
  updating_iteration_main_ =
      -1 * Eigen::Matrix<int, Eigen::Dynamic, 1>::Ones(traj_size);
  updating_iteration_backup_ =
      -1 * Eigen::Matrix<int, Eigen::Dynamic, 1>::Ones(traj_size);
  SetInitTime();
}

//============================================================================
void Mpc::Control(double* control_input) {
  // Time
  double time_now = GetTime();

  // Exit condition
  double init_time = GetInitTime();
  done_ = (time_now >= init_time + param_.ddp_.final_time_);
  if (done_) return;

  // Current step
  int current_step = floor((time_now - init_time) / param_.mpc_.dt_);

  // Current State
  state_mutex_.lock();
  init_heading_mutex_.lock();
  augmented_state_mutex_.lock();
  TwipDynamics<double>::State current_state;
  current_state << 0.25 * state_(2) - init_heading_.distance_,
      0.25 / 0.68 * 2.0 * state_(4) - init_heading_.direction_,
      state_(0) - init_heading_.tilt_, 0.25 * state_(3),
      0.25 / 0.68 * 2.0 * state_(5), state_(1), augmented_state_.x0_,
      augmented_state_.y0_;
  augmented_state_mutex_.unlock();
  init_heading_mutex_.unlock();
  state_mutex_.unlock();

  // If motion has stopped, let the main thread know, so that if it stays like
  // this for a while, we can exit
  static_ = (fabs(current_state(3)) < param_.exit_threshold_.dx_ &&
             fabs(current_state(4)) < param_.exit_threshold_.dpsi_ &&
             fabs(current_state(5)) < param_.exit_threshold_.dtheta_);

  // Read mpc control step
  int updating_iteration;
  bool mpc_reading_done = false;
  TwipDynamics<double>::Control u;
  while (!mpc_reading_done) {
    if (mpc_trajectory_main_mutex_.try_lock()) {
      u = mpc_trajectory_main_.col(current_step);
      updating_iteration = updating_iteration_main_(current_step);
      mpc_trajectory_main_mutex_.unlock();
      mpc_reading_done = true;
    } else if (mpc_trajectory_backup_mutex_.try_lock()) {
      u = mpc_trajectory_backup_.col(current_step);
      updating_iteration = updating_iteration_backup_(current_step);
      mpc_trajectory_backup_mutex_.unlock();
      mpc_reading_done = true;
    }
  }

  // Spin Torque tau_0
  double tau_0 = u(1);

  // dq
  Eigen::Vector3d dq = current_state.segment(3, 3);

  // ddq
  TwipDynamics<double> ddp_dynamics;
  DartSkeletonToTwipDynamics(three_dof_robot_, &ddp_dynamics);
  TwipDynamics<double>::State xdot = ddp_dynamics.f(current_state, u);
  double ddx = xdot(3);
  double ddpsi = xdot(4);
  double ddth = u(0);
  Eigen::Vector3d ddq;
  ddq << ddx, ddpsi, ddth;

  // A, C, Q and Gamma_fric
  TwipDynamics<double>::Forces dynamic_forces =
      ddp_dynamics.ComputeForces(current_state, u);

  // tau_1
  double tau_1 = dynamic_forces.A.row(2) * ddq;
  tau_1 += dynamic_forces.C.row(2) * dq;
  tau_1 += dynamic_forces.Q(2);
  tau_1 -= dynamic_forces.Gamma_fric(2);

  // Wheel Torques
  double tau_L = -0.5 * (tau_1 + tau_0);
  double tau_R = -0.5 * (tau_1 - tau_0);

  // Torque to current conversion
  const double motor_constant = 12.0 * 0.00706;
  const double gear_ratio = 15;
  control_input[0] = std::min(
      max_input_current_,
      std::max(-max_input_current_, tau_L / gear_ratio / motor_constant));
  control_input[1] = std::min(
      max_input_current_,
      std::max(-max_input_current_, tau_R / gear_ratio / motor_constant));

  // Log data
  writer_.SaveStep(current_state, u, time_now - init_time, updating_iteration);
}

//============================================================================
void Mpc::Destroy() {
  thread_run_mutex_.lock();
  thread_run_ = false;
  thread_run_mutex_.unlock();
  thread_->join();
  delete thread_;
}

//============================================================================
void Mpc::SetTime(double time) {
  time_mutex_.lock();
  time_ = time;
  time_mutex_.unlock();
}

//============================================================================
double Mpc::GetTime() {
  time_mutex_.lock();
  double time = time_;
  time_mutex_.unlock();
  return time;
}
