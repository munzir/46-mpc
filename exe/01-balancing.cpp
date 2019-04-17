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
 * @file 01-balancing.cpp
 * @author Munzir Zafar
 * @date Oct 29, 2018
 * @brief A neat and clean version of the original balancing logic.
 */

#include <pthread.h>  // pthread_t, pthread_mutex_init(), pthread_create()
#include <stdio.h>    // getchar()

#include <cstring>   // memset()
#include <iostream>  // std::cout, std::endl
#include <memory>    // std::make_shared

#include <krach/krach.h>               // InterfaceContext, WorldInterface
#include <krang-sim-ach/dart_world.h>  // KrangInitPoseParams, ReadInitPoseParams()
#include <somatic.h>      // has the correct order of other somatic includes
#include <dart/dart.hpp>  // dart::dynamics, dart::simulation
#include <dart/utils/urdf/urdf.hpp>  // dart::utils::DartLoader
#include <fstream>                   // std::ofstream
#include <kore.hpp>                  // Krang::Hardware

#include <somatic.pb-c.h>  // SOMATIC__: EVENT, MOTOR_PARAM; Somatic__WaistMode
#include <somatic/daemon.h>  // somatic_d: t, t_opts, _init(), _event(), _destroy()
#include <somatic/motor.h>  // somatic_motor_cmd()
#include <somatic/msg.h>  // somatic_anything_alloc(), somatic_anything_free(), Somatic_KrangPoseParams
#include <somatic/util.h>  // somatic_sig_received

#include "balancing/arms.h"  // ArmControl
#include "balancing/balancing_config.h"  // BalancingConfig, ReadConfigParams(), ReadConfigTimeStep()
#include "balancing/control.h"   // BalanceControl
#include "balancing/events.h"    // Events()
#include "balancing/joystick.h"  // Joystick
#include "balancing/keyboard.h"  // KbShared, KbHit
#include "balancing/timer.h"     // Timer
#include "balancing/torso.h"     // TorsoState, ControlTorso()
#include "balancing/waist.h"     // ControlWaist()

/* ************************************************************************* */
/// The main thread
int main(int argc, char* argv[]) {
  BalancingConfig params;

  // Ask user whether we are interfacing with simulation or hardware
  std::cout << std::endl
            << std::endl
            << "Simulation mode or hardware mode (s/h)? ";
  char key = getchar();
  if (key == 's')
    params.is_simulation_ = true;
  else if (key == 'h')
    params.is_simulation_ = false;
  else
    return 0;

  // Read config parameters
  ReadConfigParams(
      (params.is_simulation_
           ? "/usr/local/share/krang/balancing/cfg/"
             "balancing_params_simulation.cfg"
           : "/usr/local/share/krang/balancing/cfg/balancing_params.cfg"),
      &params);

  // If simulation mode, create interface to the world of simulation
  InterfaceContext* interface_context;
  WorldInterface* world_interface;
  if (params.is_simulation_) {
    interface_context = new InterfaceContext("01-balance-sim-interface");
    world_interface =
        new WorldInterface(*interface_context, "sim-cmd", "sim-state");

    // Set the initial pose of the simulation
    krang_sim_ach::dart_world::KrangInitPoseParams pose;
    krang_sim_ach::dart_world::ReadInitPoseParams(
        "/usr/local/share/krang/balancing/cfg/balancing_params_simulation.cfg",
        &pose);
    Somatic_KrangPoseParams somatic_pose;
    somatic_pose.heading = pose.heading_init;
    somatic_pose.q_base = pose.q_base_init;
    for (int i = 0; i < 3; i++) somatic_pose.xyz[i] = pose.xyz_init(i);
    somatic_pose.q_lwheel = pose.q_lwheel_init;
    somatic_pose.q_rwheel = pose.q_rwheel_init;
    somatic_pose.q_waist = pose.q_waist_init;
    somatic_pose.q_torso = pose.q_torso_init;
    for (int i = 0; i < 7; i++)
      somatic_pose.q_left_arm[i] = pose.q_left_arm_init(i);
    for (int i = 0; i < 7; i++)
      somatic_pose.q_right_arm[i] = pose.q_right_arm_init(i);
    somatic_pose.init_with_balance_pose = pose.init_with_balance_pose;
    bool success = world_interface->Reset(somatic_pose);
    if (!success) {
      // TODO: Using smart pointers will remove the need to delete explicitly
      // here
      delete world_interface;
      delete interface_context;
      return 0;
    }

    // Get the time of the simulation
    params.sim_dt_ = ReadConfigTimeStep(
        "/usr/local/share/krang-sim-ach/cfg/dart_params.cfg");
    if (params.sim_dt_ < 0.0) {
      std::cout << "Error reading time step" << std::endl;
      return 0;
    }
  }

  // Initialize the daemon
  somatic_d_opts_t dopt;
  memset(&dopt, 0, sizeof(dopt));
  dopt.ident = "01-balance";
  somatic_d_t daemon_cx;  ///< The context of the current daemon
  memset(&daemon_cx, 0, sizeof(daemon_cx));
  somatic_d_init(&daemon_cx, &dopt);

  // Load the robot
  dart::utils::DartLoader dl;
  dart::dynamics::SkeletonPtr robot;  ///< the robot representation in dart
  robot = dl.parseSkeleton(params.urdfpath);
  assert((robot != NULL) && "Could not find the robot urdf");

  // Load dart robot in dart world
  dart::simulation::WorldPtr world;  ///< the world representation in dart
  world = std::make_shared<dart::simulation::World>();
  world->addSkeleton(robot);

  // Initialize the motors and sensors on the hardware and update the kinematics
  // in dart
  int hw_mode = Krang::Hardware::MODE_AMC | Krang::Hardware::MODE_LARM |
                Krang::Hardware::MODE_RARM | Krang::Hardware::MODE_TORSO |
                Krang::Hardware::MODE_WAIST;
  Krang::Hardware*
      krang;  ///< Interface for the motor and sensors on the hardware
  krang =
      new Krang::Hardware((Krang::Hardware::Mode)hw_mode, &daemon_cx, robot);
  //    Akash made the following edits to add filter_imu option
  // bool filter_imu = (params.is_simulation_ ? false : true);
  // krang = new Krang::Hardware((Krang::Hardware::Mode)hw_mode, &daemon_cx,
  // robot,
  //                            filter_imu);

  // Create a thread that processes keyboard inputs when keys are pressed
  KbShared kb_shared;  ///< info shared by keyboard thread here
  kb_shared.kb_char_received = false;
  pthread_mutex_init(&kb_shared.kb_mutex, NULL);
  pthread_t kbhit_thread;
  pthread_create(&kbhit_thread, NULL, &KbHit, &kb_shared);

  // Constructors for other objects being used in the main loop
  Joystick joystick;
  ArmControl arm_control(&daemon_cx, krang, params);
  TorsoState torso_state;
  torso_state.mode = TorsoState::kStop;
  Somatic__WaistMode waist_mode;
  BalanceControl balance_control(krang, robot, params);
  for (int i = 0; i < robot->getNumBodyNodes(); i++) {
    dart::dynamics::BodyNodePtr body = robot->getBodyNode(i);
    std::cout << body->getName() << ": " << body->getMass() << " ";
    std::cout << body->getLocalCOM().transpose() << std::endl;
  }

  // Flag to enable wheel control. Control inputs are not sent to the wheels
  // until this flag is set
  bool start = ((params.is_simulation_ &&
                 (params.sim_init_balance_mode_ ==
                      (int)BalanceControl::BalanceMode::BAL_LO ||
                  params.sim_init_balance_mode_ ==
                      (int)BalanceControl::BalanceMode::BAL_HI ||
                  params.sim_init_balance_mode_ ==
                      (int)BalanceControl::BalanceMode::STAND ||
                  params.sim_init_balance_mode_ ==
                      (int)BalanceControl::BalanceMode::MPC))
                    ? true
                    : false);

  // Other obvioius variables
  Timer debug_timer;
  double debug_time = 0.0;
  bool debug = false;

  // Send a message to event logger; set the event code and the priority
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

  // Timing investigation
  std::ofstream time_log_file("/var/tmp/krangmpc/time_main");
  Timer main_timer;
  double main_real_dt, sim_real_dt;

  // Filtering investigation
  std::ofstream imu_log_file("/var/tmp/krangmpc/imu_log.csv");

  // Logging data for learning
  std::ofstream state_log_file("/var/tmp/krangmpc/state_log.csv");
  bool log_mark = false;  // a mark to be placed in the log file for identifying
                          // when we start trying to move 5 meters

  while (!somatic_sig_received) {
    // Decide if we want printing to happen in this iteration
    debug_time = (debug_time > 1.0
                      ? 0.0
                      : debug_time + debug_timer.ElapsedTimeSinceLastCall());
    debug = (debug_time == 0.0);

    // Read time, state and joystick inputs
    balance_control.UpdateState();
    // bool joystick_msg_received = false;
    // while (!joystick_msg_received) joystick_msg_received = joystick.Update();
    joystick.Update();

    // Decide control modes and generate control events based on keyb/joys input
    if (Events(kb_shared, joystick, &start, &balance_control, &waist_mode,
               &torso_state, &arm_control, &log_mark)) {
      // kill program if kill event was triggered
      break;
    }

    // Balancing Control
    double control_input[2];
    balance_control.BalancingController(&control_input[0],
                                        &arm_control.command_vals[0],
                                        &arm_control.more_command_vals[0]);
    if (start) {
      somatic_motor_cmd(&daemon_cx, krang->amc,
                        SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, control_input, 2,
                        NULL);
    }

    // Control the rest of the body
    arm_control.ControlArms();
    ControlWaist(waist_mode, krang);
    ControlTorso(daemon_cx, torso_state, krang);

    // If in simulation world, make the simulation time step forward
    if (params.is_simulation_) {
      Timer sim_timer;
      bool success = world_interface->Step();
      if (!success) break;
      sim_real_dt = sim_timer.ElapsedTimeSinceLastCall();
    }

    // Print the mode
    if (debug) {
      std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << std::endl;
      std::cout << "Interface: "
                << (params.is_simulation_ ? "simulation" : "hardware");
      balance_control.Print();
      std::cout << "time: " << balance_control.get_time() << std::endl;
      if (start) std::cout << "Started..." << std::endl;
    }
    if (log_mark)
      std::cout << std::endl
                << std::endl
                << "[INFO] Putting a mark in the log file" << std::endl
                << std::endl;

    // Filtering investigation
    imu_log_file << balance_control.get_time() << ", " << krang->rawImu << ", "
                 << krang->rawImuSpeed << ", " << krang->imu << ", "
                 << krang->imuSpeed << std::endl;

    // Logging data for learning
    if (log_mark)
      state_log_file << Eigen::Matrix<double, 1, 29>::Zero() << std::endl;
    state_log_file << balance_control.get_time() << " "
                   << balance_control.get_mode() << " " << krang->rawImu << " "
                   << krang->imu << " " << krang->rawImuSpeed << " "
                   << krang->imuSpeed << " "
                   << balance_control.get_com().transpose() << " "
                   << balance_control.get_state().transpose() << " "
                   << balance_control.get_ref_state().transpose() << " "
                   << balance_control.get_pd_gains().transpose() << " "
                   << control_input[0] << " " << control_input[1] << std::endl;

    // Timing investigation
    main_real_dt = main_timer.ElapsedTimeSinceLastCall();
    if (balance_control.get_mode() == BalanceControl::MPC) {
      time_log_file << main_real_dt << ", " << sim_real_dt << std::endl;
    }

    // Wait till control loop's desired period ends
    if (!params.is_simulation_) {
      auto main_usecs = (unsigned int)(main_real_dt * 1e6);
      if (main_usecs < params.control_period_)
        usleep(params.control_period_ - main_usecs);
      main_timer.ElapsedTimeSinceLastCall();  // reset the timer
    }
  }

  // Send the stoppig event
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

  std::cout << "destroying" << std::endl;
  time_log_file.close();
  imu_log_file.close();
  state_log_file.close();
  delete krang;
  if (params.is_simulation_) {
    delete world_interface;
    delete interface_context;
  }
  somatic_d_destroy(&daemon_cx);
  return 0;
}
