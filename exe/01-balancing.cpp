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

#include <krach/krach.h>  // InterfaceContext, WorldInterface
#include <somatic.h>      // has the correct order of other somatic includes
#include <dart/dart.hpp>  // dart::dynamics, dart::simulation
#include <dart/utils/urdf/urdf.hpp>  // dart::utils::DartLoader
#include <kore.hpp>                  // Krang::Hardware

#include <somatic.pb-c.h>  // SOMATIC__: EVENT, MOTOR_PARAM; Somatic__WaistMode
#include <somatic/daemon.h>  // somatic_d: t, t_opts, _init(), _event(), _destroy()
#include <somatic/motor.h>  // somatic_motor_cmd()
#include <somatic/msg.h>    // somatic_anything_alloc(), somatic_anything_free()
#include <somatic/util.h>   // somatic_sig_received

#include "balancing/arms.h"  // ArmControl
#include "balancing/balancing_config.h"  // BalancingConfig, ReadConfigParams(), ReadConfigTimeStep()
#include "balancing/control.h"   // BalancingControl
#include "balancing/events.h"    // Events()
#include "balancing/joystick.h"  // Joystick
#include "balancing/keyboard.h"  // KbShared, KbHit
#include "balancing/torso.h"     // TorsoState, ControlTorso()
#include "balancing/waist.h"     // ControlWaist()

/* ************************************************************************* */
/// The main thread
int main(int argc, char* argv[]) {
  // Read config parameters
  BalancingConfig params;
  ReadConfigParams("../cfg/balancing_params.cfg", &params);

  // Ask user whether we are interfacing with simulation or hardware
  std::cout << std::endl
            << std::endl
            << "Simulation mode or hardware mode (s/h)? ";
  char key = getchar();
  bool is_simulation;
  if (key == 's')
    is_simulation = true;
  else if (key == 'h')
    is_simulation = false;
  else
    return 0;

  // If simulation mode, create interface to the world of simulation
  InterfaceContext* interface_context;
  WorldInterface* world_interface;
  double sim_dt;
  if (is_simulation) {
    interface_context = new InterfaceContext("01-balance-sim-interface");
    world_interface =
        new WorldInterface(*interface_context, "sim-cmd", "sim-state");
    sim_dt = ReadConfigTimeStep(
        "/usr/local/share/krang-sim-ach/cfg/dart_params.cfg");
    if (sim_dt < 0.0) {
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

  // Flag to enable wheel control. Control inputs are not sent to the wheels
  // until this flag is set
  bool start = false;

  // Other obvioius variables
  size_t debug_iter = 0;
  double time = 0.0;

  // Send a message to event logger; set the event code and the priority
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

  while (!somatic_sig_received) {
    bool debug = (debug_iter++ % 20 == 0);

    // Read time, state and joystick inputs
    time += balance_control.ElapsedTimeSinceLastCall();
    balance_control.UpdateState();
    bool joystick_msg_received = false;
    while (!joystick_msg_received) joystick_msg_received = joystick.Update();

    // Decide control modes and generate control events based on keyb/joys input
    if (Events(kb_shared, joystick, &start, &balance_control, &waist_mode,
               &torso_state, &arm_control)) {
      // kill program if kill event was triggered
      break;
    }

    // Balancing Control
    double control_input[2];
    balance_control.BalancingController(&control_input[0]);
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
    if (is_simulation) {
      bool success = world_interface->Step();
      if (!success) break;
    }

    // Print the mode
    if (debug) {
      std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << std::endl;
      std::cout << "Interface: " << (is_simulation ? "simulation" : "hardware");
      balance_control.Print();
      if (start) std::cout << "Started..." << std::endl;
    }
  }

  // Send the stoppig event
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

  std::cout << "destroying" << std::endl;
  delete krang;
  if (is_simulation) {
    delete interface_context;
    delete world_interface;
  }
  somatic_d_destroy(&daemon_cx);
  return 0;
}
