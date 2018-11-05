/**
 * @file 01-balancing.cpp
 * @author Munzir Zafar
 * @date Oct 29, 2018
 * @brief A neat and clean version of the original balancing logic.
 */

#include "balancing_config.h"
#include "arms.h"
#include "waist.h"
#include "grippers.h"
#include "torso.h"
#include "joystick.h"
#include "keyboard.h"
#include "control.h"
#include "events.h"
#include "kore/display.hpp"
#include "../../18h-Util/lqr.hpp"
#include "../../18h-Util/adrc.hpp"
#include "file_ops.hpp"
#include "utils.h"
#include <dart/utils/urdf/urdf.hpp>

kbShared kb_shared;

/// The vector of states
//vector <LogState*> logStates;

// Debug flags default values
bool debugGlobal = false, logGlobal = true;
dart::simulation::WorldPtr world;			///< the world representation in dart
bool start = false;						///< Giving time to the user to get the robot in balancing angle

/* ******************************************************************************************** */
typedef Eigen::Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Eigen::Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Eigen::Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

/* ******************************************************************************************** */
// Globals for imu, motors and joystick

somatic_d_t daemon_cx;				///< The context of the current daemon

Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
dart::dynamics::SkeletonPtr robot;			///< the robot representation in dart

bool joystickControl = false;

char b [10];						///< Stores the joystick button inputs
double x [6];


void run (BalancingConfig& params) {

  // Send a message; set the event code and the priority
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
      SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

  size_t debug_iter = 0;
  double time = 0.0;
  ArmState arm_state;
  arm_state.mode = ArmState::kStop;
  TorsoState torso_state;
  torso_state.mode = TorsoState::kStop;
  Somatic__WaistMode waist_mode;
  BalanceControl balance_control(krang, robot, params);
  while(!somatic_sig_received) {

    bool debug = (debug_iter++ % 20 == 0);
    debugGlobal = debug;

    // Read time, state and joystick inputs
    time += balance_control.ElapsedTimeSinceLastCall();
    balance_control.UpdateState();
    bool gotInput = false;
    while(!gotInput) gotInput = getJoystickInput(b, x);

    // Process events based on joystick/keybaord input
    keyboardEvents(kb_shared, start, joystickControl, balance_control);
    joystickBalancingEvents(b, x, joystickControl, balance_control);
    if(joystickControl) {
      joyStickArmEvents(b, x, &arm_state);
      waist_mode = joystickWaistEvents(x[5]);
      joystickTorsoEvents(b, x, &torso_state);
    }
    if (!joystickKillEvent(b)) {
      break;
    }

    // Balancing Control
    double control_input[2];
    balance_control.BalancingController(joystickControl, &control_input[0]);
    if(start) {
      somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,
                        control_input, 2, NULL);
    }

    // Control the rest of the body
    if(joystickControl) {
      controlArms(daemon_cx, arm_state, krang);
      controlWaist(waist_mode, krang);
      controlTorso(daemon_cx, torso_state, krang);
    }

    // Print the information about the last iteration
      // (after reading effects of it from sensors)
      // NOTE: Constructor order is NOT the print order
      // if(logGlobal) {
      //  logStates.push_back(new LogState(time, com, averagedTorque,
      //                                   externalWrench(4), krang->amc->cur[0],
      //                                   krang->amc->cur[1], state, refState,
      //                                   lastUleft, lastUright));
      //}

    // Print the mode
    if(debug) {
      std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << std::endl;
      balance_control.Print();
      if(start)
        std::cout << "Started..." << std::endl;
      if(joystickControl)
        std::cout << "Joystick for Arms and Waist..." << std::endl;
    }
  }

  // Send the stoppig event
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
           SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
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

  // Initialize the motors and sensors on the hardware and update the kinematics in dart
  int hwMode = Krang::Hardware::MODE_AMC | Krang::Hardware::MODE_LARM |
    Krang::Hardware::MODE_RARM | Krang::Hardware::MODE_TORSO | Krang::Hardware::MODE_WAIST;
  krang = new Krang::Hardware((Krang::Hardware::Mode) hwMode, &daemon_cx, robot);

  // Initialize the joystick channel
  openJoystickChannel();

  // Create a thread to wait for user input to begin balancing
  kb_shared.kb_char_received = false;
  pthread_mutex_init(&kb_shared.kb_mutex, NULL);
  pthread_t kbhitThread;
  pthread_create(&kbhitThread, NULL, &kbhit, &kb_shared);
}

/* ******************************************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon structs.
void destroy() {

  cout << "destroying" << endl;

  // ===========================
  // Stop motors, close motor/sensor channels and destroy motor objects

  // To prevent arms from halting if joystick control is not on, change mode of krang
  if(!joystickControl) {
    somatic_motor_destroy(&daemon_cx, krang->arms[Krang::LEFT]);
    somatic_motor_destroy(&daemon_cx, krang->arms[Krang::RIGHT]);
    krang->arms[Krang::LEFT] = NULL;
    krang->arms[Krang::RIGHT] = NULL;
  }
  delete krang;

  // Destroy the daemon resources
  somatic_d_destroy(&daemon_cx);

  // Print the data
  //printf("log states size: %lu\n", logStates.size());
  //for(size_t i = 0; i < logStates.size(); i++) {
  //  logStates[i]->print();
  //  delete logStates[i];
  //}
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

  BalancingConfig params;
  ReadConfigParams("../params.cfg", &params);

  // Debug options from command line
  debugGlobal = 1; logGlobal = 0;
  if(argc == 8) {
    if(argv[7][0]=='l') { debugGlobal = 0; logGlobal = 1;}
    else if(argv[7][0] == 'd') {debugGlobal = 1; logGlobal = 0; }
  }

  getchar();

  // Initialize, run, destroy
  init(params);
  run(params);
  destroy();
  return 0;
}
