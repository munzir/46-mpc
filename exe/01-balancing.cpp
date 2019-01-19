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

#include <somatic.h>      // has the correct order of other somatic includes
#include <dart/dart.hpp>  // dart::dynamics, dart::simulation
#include <dart/utils/urdf/urdf.hpp>  // dart::utils::DartLoader
#include <kore.hpp>                  // Krang::Hardware

#include <somatic.pb-c.h>  // SOMATIC__: EVENT, MOTOR_PARAM; Somatic__WaistMode
#include <somatic/daemon.h>  // somatic_d: t, t_opts, _init(), _event(), _destroy()
#include <somatic/motor.h>  // somatic_motor_cmd()
#include <somatic/msg.h>    // somatic_anything_alloc(), somatic_anything_free()
#include <somatic/util.h>   // somatic_sig_received

#include "arms.h"              //ArmControl
#include "balancing_config.h"  // BalancingConfig, ReadConfigParams()
#include "control.h"           // BalancingControl
#include "events.h"            // Events()
#include "joystick.h"          // Joystick
#include "keyboard.h"          // kbShared
#include "torso.h"             // TorsoState
#include "waist.h"             // controlWaist()

/* ****************************************************************************
 */
// Globals for imu, motors and joystick

somatic_d_t daemon_cx;  ///< The context of the current daemon

Krang::Hardware*
    krang;  ///< Interface for the motor and sensors on the hardware
dart::dynamics::SkeletonPtr robot;  ///< the robot representation in dart

char b[10];  ///< Stores the joystick button inputs
double x[6];

kbShared kb_shared;  ///< info shared by keyboard thread here

// Debug flags default values
dart::simulation::WorldPtr world;  ///< the world representation in dart
bool start = false;                ///< Giving time to the user to get the robot
                                   ///  in balancing angle

/* ****************************************************************************
 */
// The infinite while loop lives here
void run(BalancingConfig& params) {
  // Send a message; set the event code and the priority
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

  size_t debug_iter = 0;
  double time = 0.0;
  Joystick joystick;
  ArmControl arm_control(&daemon_cx, krang, params);
  TorsoState torso_state;
  torso_state.mode = TorsoState::kStop;
  Somatic__WaistMode waist_mode;
  BalanceControl balance_control(krang, robot, params);
  while (!somatic_sig_received) {
    bool debug = (debug_iter++ % 20 == 0);

    // Read time, state and joystick inputs
    time += balance_control.ElapsedTimeSinceLastCall();
    balance_control.UpdateState();
    bool gotInput = false;
    while (!gotInput) gotInput = joystick.Update();

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
    controlWaist(waist_mode, krang);
    controlTorso(daemon_cx, torso_state, krang);

    // Print the mode
    if (debug) {
      std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << std::endl;
      balance_control.Print();
      if (start) std::cout << "Started..." << std::endl;
    }
  }

  // Send the stoppig event
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* *********************************************************************************
 */
/// Initialize the motor and daemons
void init(BalancingConfig& params) {
  // Initialize the daemon
  somatic_d_opts_t dopt;
  memset(&dopt, 0, sizeof(dopt));
  dopt.ident = "01-balance";
  somatic_d_init(&daemon_cx, &dopt);

  // Load the robot
  dart::utils::DartLoader dl;
  robot = dl.parseSkeleton(params.urdfpath);
  assert((robot != NULL) && "Could not find the robot urdf");

  // Load dart robot in dart world
  world = std::make_shared<dart::simulation::World>();
  world->addSkeleton(robot);

  // Initialize the motors and sensors on the hardware and update the kinematics
  // in dart
  int hwMode = Krang::Hardware::MODE_AMC | Krang::Hardware::MODE_LARM |
               Krang::Hardware::MODE_RARM | Krang::Hardware::MODE_TORSO |
               Krang::Hardware::MODE_WAIST;
  krang = new Krang::Hardware((Krang::Hardware::Mode)hwMode, &daemon_cx, robot);

  // Create a thread to wait for user input to begin balancing
  kb_shared.kb_char_received = false;
  pthread_mutex_init(&kb_shared.kb_mutex, NULL);
  pthread_t kbhitThread;
  pthread_create(&kbhitThread, NULL, &kbhit, &kb_shared);
}

/* *********************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon
/// structs.
void destroy() {
  std::cout << "destroying" << std::endl;

  delete krang;

  somatic_d_destroy(&daemon_cx);
}

/* ************************************************************************* */
/// The main thread
int main(int argc, char* argv[]) {
  BalancingConfig params;
  ReadConfigParams("../params.cfg", &params);

  // Wait for a character input before starting the program
  getchar();

  // Initialize, run, destroy
  init(params);
  run(params);
  destroy();
  return 0;
}
