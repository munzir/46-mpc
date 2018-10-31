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
#include "helpers.h"
#include "kore/display.hpp"
#include "../../18h-Util/lqr.hpp"
#include "../../18h-Util/adrc.hpp"
#include "file_ops.hpp"
#include "utils.h"

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
// If a character was entered from the keyboard process it

void keyboardEvents() {

  char input;

  if(kbCharReceived(&input)) {

    if(input=='s') start = true;
    //else if(input=='.') readGains();
    else if(input=='j') {
      joystickControl = !joystickControl;
      if(joystickControl == true) {
        somatic_motor_reset(&daemon_cx, krang->arms[Krang::LEFT]);
        somatic_motor_reset(&daemon_cx, krang->arms[Krang::RIGHT]);
      }
    }
    else if(input=='1') {
      printf("Mode 1\n");
      K = K_groundLo;
      MODE = GROUND_LO;
    }
    else if(input=='2') {
      printf("Mode 2\n");
      K = K_stand;
      MODE = STAND;
    }
    else if(input=='3') {
      printf("Mode 3\n");
      K = K_sit;
      MODE = SIT;
    }
    else if(input=='4') {
      printf("Mode 4\n");
      K = K_balLow;
      MODE = BAL_LO;
    }
    else if(input=='5') {
      printf("Mode 5\n");
      K = K_balHigh;
      MODE = BAL_HI;
    }
    else if(input=='6') {
      printf("Mode 6\n");
      K = K_groundHi;
      MODE = GROUND_HI;
    }
  }
}

/* ************************************************************************************/
/// Changes desired arm state based on joystick input
void joystickTorsoEvents(const char* b, const double* x, TorsoState* torso_state) {

  if(fabs(x[4]) < 0.1)
    torso_state->mode = TorsoState::kStop;
  else {
    torso_state->mode = TorsoState::kMove;
    torso_state->command_val = x[4] / 7.0;
  }

}

/* ************************************************************************************/
/// Changes desired arm state based on joystick input
void joyStickArmEvents(const char* b, const double* x, ArmState* arm_state) {

  // Return if the x[3] is being used for robotiq hands
  if(fabs(x[3]) > 0.2) arm_state->mode = ArmState::kStop;

  // Check if one of the preset configurations are requested by pressing 9 and
  // any of the buttons from 1 to 4 at the same time
  if(((b[4] == 1) && (b[6] == 1)) || ((b[5] == 1) && (b[7] == 1))) {

    // Check if the button is pressed for the arm configuration is pressed, if so send pos commands
    bool noConfs = true;
    for(size_t i = 0; i < 4; i++) {
      if(b[i] == 1) {
        if((b[4] == 1) && (b[6] == 1) && (b[5] == 1) && (b[7] == 1)) {
          arm_state->mode = ArmState::kMoveBothToPresetPos;
          arm_state->preset_config_num = i;
        }
        else if((b[4] == 1) && (b[6] == 1)) {
          arm_state->mode = ArmState::kMoveLeftToPresetPos;
          arm_state->preset_config_num = i;
        }
        else if((b[5] == 1) && (b[7] == 1))  {
          arm_state->mode = ArmState::kMoveRightToPresetPos;
          arm_state->preset_config_num = i;
        }
        noConfs = false;
      }
    }

    // If nothing is pressed, stop the arms
    if(noConfs) {
      arm_state->mode = ArmState::kStop;
    }
  }
  // If only one of the front buttons is pressed
  else {
    if(b[4] && !b[6]) {
      arm_state->mode = ArmState::kMoveLeftBigSet;
      for(int i = 0; i < 4; i++)
        arm_state->command_vals[i] = x[i];
    } else if(!b[4] && b[6]) {
      arm_state->mode = ArmState::kMoveLeftSmallSet;
      for(int i = 4; i < 7; i++)
        arm_state->command_vals[i] = x[i-4];
    } else if(b[5] && !b[7]) {
      arm_state->mode = ArmState::kMoveRightBigSet;
      for(int i = 0; i < 4; i++)
        arm_state->command_vals[i] = x[i];
    } else if(!b[5] && b[7]) {
      arm_state->mode = ArmState::kMoveRightSmallSet;
      for(int i = 4; i < 7; i++)
        arm_state->command_vals[i] = x[i-4];
    }
    else {
      arm_state->mode = ArmState::kStop;
    }
  }
}

/* ********************************************************************************************* */
// Decides what mode waist should be in based on the value of the input argument that is assumed
// to be one of the axes of the joystick
Somatic__WaistMode joystickWaistEvents(double x) {
  // Set the mode we want to send to the waist daemon
  Somatic__WaistMode waistMode;
  if(x < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
  else if(x > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
  else waistMode = SOMATIC__WAIST_MODE__STOP;

  return waistMode;
}

/* ********************************************************************************************* */
/// Handles the wheel commands if we are started
void controlWheels(somatic_d_t& daemon_cx_, bool joystickControl_,
                   KRANG_MODE MODE_,  Vector6d& K_, Vector6d& error, bool debug,
                   double& lastUleft, double& lastUright,
                   Krang::Hardware* krang_) {

  // Compute the current
  double u_theta = K_.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());
  double u_x = K_(2)*error(2) + K_(3)*error(3);
  double u_spin =  -K_.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
  u_spin = max(-30.0, min(30.0, u_spin));

  // Compute the input for left and right wheels
  if(joystickControl_ && ((MODE_ == GROUND_LO) || (MODE_ == GROUND_HI))) {u_x = 0.0; u_spin = 0.0;}
  double input [2] = {u_theta + u_x + u_spin, u_theta + u_x - u_spin};
  input[0] = max(-49.0, min(49.0, input[0]));
  input[1] = max(-49.0, min(49.0, input[1]));
  if(debug) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
  lastUleft = input[0], lastUright = input[1];

  if(start) {
    if(debug) cout << "Started..." << endl;
    somatic_motor_cmd(&daemon_cx_, krang_->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
  }
}

/* ********************************************************************************************* */
/// Update Krang Mode based on configuration, state and state error, updates the K matrices used to calculate u/
void updateKrangMode(Vector6d& error, size_t& mode4iter, Vector6d& state) {
  size_t mode4iterLimit = 100;
  // If in ground Lo mode and waist angle increases beyond 150.0 goto groundHi mode
  if(MODE == GROUND_LO) {
    if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 < 150.0*M_PI/180.0) {
      MODE = GROUND_HI;
      K = K_groundHi;
    }
  }
    // If in ground Hi mode and waist angle decreases below 150.0 goto groundLo mode
  else if(MODE == GROUND_HI) {
    if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
      MODE = GROUND_LO;
      K = K_groundLo;
    }
  }

    // If we are in the sit down mode, over write the reference
  else if(MODE == SIT) {
    static const double limit = ((-103.0 / 180.0) * M_PI);
    if(krang->imu < limit) {
      printf("imu (%lf) < limit (%lf): changing to mode 1\n", krang->imu, limit);
      MODE = GROUND_LO;
      K = K_groundLo;
    }
    else error(0) = krang->imu - limit;
  }
    // if in standing up mode check if the balancing angle is reached and stayed, if so switch to balLow mode
  else if(MODE == STAND) {
    if(fabs(state(0)) < 0.034) mode4iter++;
    // Change to mode 4 (balance low) if stood up enough
    if(mode4iter > mode4iterLimit) {
      MODE = BAL_LO;
      mode4iter = 0;
      K = K_balLow;
    }
  }
    // COM error correction in balLow mode
  else if(MODE == BAL_LO) {
    // error(0) += 0.005;
  }
    // COM error correction in balHigh mode
  else if(MODE == BAL_HI) {
    // error(0) -= 0.005;
  }
}

/* ********************************************************************************************* */
/// Handles the wheel commands if we are started
bool controlStandSit(char* b_, Vector6d& error, Vector6d& state,
                     BalancingConfig& params, KRANG_MODE& MODE_,
                     Vector6d& K_) {
  // ==========================================================================
  // Quit if button 9 on the joystick is pressed, stand/sit if button 10 is pressed
  // Quit

  static bool b9Prev = 0;

  if(b_[8] == 1) return false;

    // Stand/Sit if button 10 is pressed and conditions are right
  else if(b9Prev == 0 && b_[9] == 1) {

    // If in ground mode and state error is not high stand up
    if(MODE_ == GROUND_LO) {
      if(state(0) < 0.0 && error(0) > -10.0*M_PI/180.0) {
        printf("\n\n\nMode 2\n\n\n");
        K_ = params.pdGainsStand;
        MODE_ = STAND;
      } else {
        printf("\n\n\nCan't stand up, balancing error is too high!\n\n\n");
      }
    }

      // If in balLow mode and waist is not too high, sit down
    else if(MODE_ == STAND || MODE_ == BAL_LO) {
      if((krang->waist->pos[0] - krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
        printf("\n\n\nMode 3\n\n\n");
        K_ = params.pdGainsSit;
        MODE_ = SIT;
      } else {
        printf("\n\n\nCan't sit down, Waist is too high!\n\n\n");
      }
    }
  }
  b9Prev = b_[9];
  return true;
}

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
    keyboardEvents();
    joystickEvents(b, x, params, jsFwdAmp, jsSpinAmp, joystickControl, MODE, K,
                   js_forw, js_spin);
    if(debug) cout << "js_forw: " << js_forw << ", js_spin: " << js_spin << endl;
    if(joystickControl) {
      if(debug) cout << "Joystick for Arms and Waist..." << endl;
      joyStickArmEvents(b, x, &arm_state);
      waist_mode = joystickWaistEvents(x[5]);
      joystickTorsoEvents(b, x, &torso_state);
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
    updateReference(js_forw, js_spin, dt, refState);
    if(debug) cout << "refState: " << refState.transpose() << endl;

    // Calculate state Error
    error = state - refState;
    if(debug) cout << "error: " << error.transpose() << ", imu: " << krang->imu / M_PI * 180.0 << endl;

    // Krang mode events
    updateKrangMode(error, mode4iter, state);

    // Stand/Sit events
    if(!controlStandSit(b, error, state, params, MODE, K)) {
      break;
    }

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
    controlWheels(daemon_cx, joystickControl, MODE, K, error, debug,
                  lastUleft, lastUright, krang);

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
    if(debug) printf("Mode : %d\tdt: %lf\n", MODE, dt);
  }

  // Send the stoppig event
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
           SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
// // Change robot's beta values (parameters)
SkeletonPtr setParameters(SkeletonPtr robot_, Eigen::MatrixXd betaParams, int bodyParams) {
  Eigen::Vector3d bodyMCOM;
  double mi;
  int numBodies = betaParams.cols()/bodyParams;
  for (int i = 0; i < numBodies; i++) {
    mi = betaParams(0, i * bodyParams);
    bodyMCOM(0) = betaParams(0, i * bodyParams + 1);
    bodyMCOM(1) = betaParams(0, i * bodyParams + 2);
    bodyMCOM(2) = betaParams(0, i * bodyParams + 3);

    robot_->getBodyNode(i)->setMass(mi);
    robot_->getBodyNode(i)->setLocalCOM(bodyMCOM/mi);
  }
  return robot_;
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


  // Initialize the joystick channel
  openJoystickChannel();

  // Create a thread to wait for user input to begin balancing
  pthread_mutex_init(&kb_mutex, NULL);
  pthread_t kbhitThread;
  pthread_create(&kbhitThread, NULL, &kbhit, NULL);
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
