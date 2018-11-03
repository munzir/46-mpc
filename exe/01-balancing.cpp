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
vector <LogState*> logStates;

// Debug flags default values
bool debugGlobal = false, logGlobal = true;
dart::simulation::WorldPtr world;			///< the world representation in dart
bool start = false;						///< Giving time to the user to get the robot in balancing angle
/* ******************************************************************************************** */
// LQR hack ratios

Eigen::MatrixXd lqrHackRatios;

/* ******************************************************************************************** */
// Constants for the robot kinematics
const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches

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

double jsFwdAmp;				///< The gains for joystick forward/reverse input
double jsSpinAmp;				///< The gains for joystick left/right spin input

char b [10];						///< Stores the joystick button inputs
double x [6];


/* ******************************************************************************************** */
// All the freaking gains

KRANG_MODE MODE = GROUND_LO;
Vector6d K_groundLo;
Vector6d K_groundHi;
Eigen::Vector2d J_ground (1.0, 1.0);
Vector6d K_stand;
Eigen::Vector2d J_stand;
Vector6d K_sit;
Eigen::Vector2d J_sit;
Vector6d K_balLow;
Eigen::Vector2d J_balLow;
Vector6d K_balHigh;
Eigen::Vector2d J_balHigh;
Vector6d K = K_groundLo;

/* ******************************************************************************************** */
// Constants for end-effector wrench estimation
//static const double eeMass = 2.3 + 0.169 + 0.000;			///< The mass of the robotiq end-effector
static const double eeMass = 1.6 + 0.169 + 0.000;			///< The mass of the Schunk end-effector
static const Eigen::Vector3d s2com (0.0, -0.008, 0.091); // 0.065 robotiq itself, 0.026 length of ext + 2nd


/* ******************************************************************************************** */
/// The continuous control loop which has 4 state variables, {x, x., psi, psi.}, where
/// x is for the position along the heading direction and psi is the heading angle. We create
/// reference x and psi values from the joystick and follow them with pd control.
void run (BalancingConfig& params) {

  // Send a message; set the event code and the priority
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
      SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

  // Initially the reference position and velocities are zero (don't move!) (and error!)
  // Initializing here helps to print logs of the previous state
  Vector6d refState = Vector6d::Zero(), state = Vector6d::Zero(), error = Vector6d::Zero();

  // Read the FT sensor wrenches, shift them on the wheel axis and display
  size_t c_ = 0;
  struct timespec t_now, t_prev = aa_tm_now();
  double time = 0.0;
  Vector6d externalWrench;
  Eigen::Vector3d com;

  // Initialize the running history
  const size_t historySize = 60;

  // Torque to current conversion
  // Motor Constant
  // TODO: Copy comments from repo 28 mpc branch
  double km = 12.0 * 0.00706;

  // Gear Ratio
  double GR = 15;

  // Continue processing data until stop received
  double js_forw = 0.0, js_spin = 0.0, averagedTorque = 0.0, lastUleft = 0.0, lastUright = 0.0;
  size_t mode4iter = 0;
  KRANG_MODE lastMode = MODE; bool lastStart = start;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4,4);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4,1);
  Eigen::VectorXd B_thWheel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd B_thCOM = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd LQR_Gains = Eigen::VectorXd::Zero(4);

  ArmState arm_state;
  arm_state.mode = ArmState::kStop;
  TorsoState torso_state;
  torso_state.mode = TorsoState::kStop;
  Somatic__WaistMode waist_mode;

  const char MODE_string[][16] = {"Ground Lo", "Stand", "Sit", "Bal Lo", "Bal Hi"};

  while(!somatic_sig_received) {

    bool debug = (c_++ % 20 == 0);
    debugGlobal = debug;
    if(debug) cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << endl;

    // ========================================================================
    // Get time, state, joystick (and keybaord) readings

    // Get the current time and compute the time difference and update the prev. time
    t_now = aa_tm_now();
    double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));
    t_prev = t_now;
    time += dt;

    // Get the current state and ask the user if they want to start
    getState(krang, robot, state, dt, &com);
    if(debug) {
      cout << "\nstate: " << state.transpose() << endl;
      cout << "com: " << com.transpose() << endl;
      cout << "WAIST ANGLE: " << krang->waist->pos[0] << endl;
    }

    // Get the joystick input
    bool gotInput = false;
    while(!gotInput) gotInput = getJoystickInput(b, x);

    // ========================================================================
    // Generate events based on keyboard input, joystick input and state

    // Process keyboard and joystick events
    keyboardEvents(kb_shared, params, start, joystickControl, daemon_cx, krang, K, MODE);
    joystickEvents(daemon_cx, krang, b, x, params, joystickControl, MODE, K,
                   js_forw, js_spin);
    if(debug) cout << "js_forw: " << js_forw << ", js_spin: " << js_spin << endl;
    if(joystickControl) {
      if(debug) cout << "Joystick for Arms and Waist..." << endl;
      joyStickArmEvents(b, x, &arm_state);
      waist_mode = joystickWaistEvents(x[5]);
      joystickTorsoEvents(b, x, &torso_state);
    }

    // Stand/Sit events
    if (!controlStandSit(b, krang, state, error, params, MODE, K)) {
      break;
    }

//  // Cancel any position built up in previous mode
//  if(lastMode != MODE) {
//    refState(2) = state(2), refState(4) = state(4);
//    lastMode = MODE;
//  }
//  if(lastStart != start) {
//    refState(2) = state(2), refState(4) = state(4);
//    lastStart = start;
//  }

    // Update the reference values for the position and spin
    updateReference (params, MODE, js_forw, js_spin, jsFwdAmp, jsSpinAmp, dt,
                     refState);
    if(debug) cout << "refState: " << refState.transpose() << endl;

    // Calculate state Error
    error = state - refState;
    if(debug) cout << "error: " << error.transpose() << ", imu: " << krang->imu / M_PI * 180.0 << endl;

    // Krang mode events
    updateKrangMode(state, krang, params, error, mode4iter, MODE, K);


    // ========================================================================
    // Balancing Control

    // linearize dynamics
    computeLinearizedDynamics(robot, A, B, B_thWheel, B_thCOM);

    // lqr hack ratios only on first iteration
    if (c_ == 1) {
      // Call lqr to compute hack ratios
      lqr(A, B, params.lqrQ, params.lqrR, LQR_Gains);
      LQR_Gains /= (GR * km);

      lqrHackRatios = Eigen::MatrixXd::Zero(4, 4);
      for (int i = 0; i < lqrHackRatios.cols(); i++) {

        lqrHackRatios(i, i) = K_stand(i) / -LQR_Gains(i);

      }
    }

    // lqr gains
    lqr(A, B, params.lqrQ, params.lqrR, LQR_Gains);
    LQR_Gains /= (GR * km);
    LQR_Gains = lqrHackRatios * LQR_Gains;
    if(MODE == STAND || MODE == BAL_LO || MODE == BAL_HI) {
        K.head(4) = -LQR_Gains;
    }
    if(debug) cout << "lqr gains" << LQR_Gains.transpose() << endl;
    if(debug) cout << "K: " << K.transpose() << endl;

    // send commands to wheel motors
    double control_input[2];
    BalanceControl(daemon_cx, start, joystickControl, MODE, K, error, debug,
                   &control_input[0]);
    lastUleft = control_input[0], lastUright = control_input[1];
    if(start) {
      if(debug) std::cout << "Started..." << std::endl;
      somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,
                        control_input, 2, NULL);
    }


    // ========================================================================
    // Control the rest of the body
    if(joystickControl) {
      controlArms(daemon_cx, arm_state, krang);
      controlWaist(waist_mode, krang);
      controlTorso(daemon_cx, torso_state, krang);
    }

    // ========================================================================
    // Print the information about the last iteration (after reading effects of it from sensors)
    // NOTE: Constructor order is NOT the print order
    if(logGlobal) {
      logStates.push_back(new LogState(time, com, averagedTorque,
                                       externalWrench(4), krang->amc->cur[0],
                                       krang->amc->cur[1], state, refState,
                                       lastUleft, lastUright));
    }

    // Print the mode
    if(debug) printf("Mode : %s\tdt: %lf\n", MODE_string[MODE], dt);
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

  // Read CoM estimation model paramters
  Eigen::MatrixXd beta;
  string inputBetaFilename = params.comParametersPath;
  try {
    cout << "Reading converged beta ...\n";
    beta = readInputFileAsMatrix(inputBetaFilename);
    cout << "|-> Done\n";
  } catch (exception& e) {
    cout << e.what() << endl;
    assert(false && "Problem loading CoM parameters...");
  }
  robot = setParameters(robot, beta, 4);

  // Initialize the gains
  K_groundLo = params.pdGainsGroundLo; J_ground = params.joystickGainsGroundLo;
  K_groundHi = params.pdGainsGroundHi; J_ground = params.joystickGainsGroundHi;
  K_stand = params.pdGainsStand;       J_stand = params.joystickGainsStand;
  K_sit = params.pdGainsSit;           J_sit = params.joystickGainsSit;
  K_balLow = params.pdGainsBalLo;      J_balLow = params.joystickGainsBalLo;
  K_balHigh = params.pdGainsBalHi;     J_balHigh = params.joystickGainsBalHi;
  K = K_groundLo;

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
  printf("log states size: %lu\n", logStates.size());
  for(size_t i = 0; i < logStates.size(); i++) {
    logStates[i]->print();
    delete logStates[i];
  }
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
