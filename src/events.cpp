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
 * @file events.cpp
 * @author Munzir Zafar
 * @date Oct 31, 2018
 * @brief Maps input/state events to desired control states/modes for the robot
 */
#include "events.h"

#include "arms.h"
#include "balancing_config.h"
#include "control.h"
#include <Eigen/Eigen>
#include <kore.hpp>
#include <somatic.h>
#include "torso.h"

/* ******************************************************************************************** */
// If a character was entered from the keyboard process it

void keyboardEvents(kbShared& kb_shared, const BalancingConfig& params, bool& start_,
                    bool& joystickControl_, somatic_d_t& daemon_cx_,
                    Krang::Hardware* krang_, Eigen::Matrix<double, 6, 1>& K_,
                    KRANG_MODE& MODE_) {

  char input;

  if(kbCharReceived(kb_shared, &input)) {

    if(input=='s') start_ = true;
    //else if(input=='.') readGains();
    else if(input=='j') {
      joystickControl_ = !joystickControl_;
      if(joystickControl_ == true) {
        somatic_motor_reset(&daemon_cx_, krang_->arms[Krang::LEFT]);
        somatic_motor_reset(&daemon_cx_, krang_->arms[Krang::RIGHT]);
      }
    }
    else if(input=='1') {
      printf("Mode 1\n");
      K_ = params.pdGainsGroundLo;
      MODE_ = GROUND_LO;
    }
    else if(input=='2') {
      printf("Mode 2\n");
      K_ = params.pdGainsStand;
      MODE_ = STAND;
    }
    else if(input=='3') {
      printf("Mode 3\n");
      K_ = params.pdGainsSit;
      MODE_ = SIT;
    }
    else if(input=='4') {
      printf("Mode 4\n");
      K_ = params.pdGainsBalLo;
      MODE_ = BAL_LO;
    }
    else if(input=='5') {
      printf("Mode 5\n");
      K_ = params.pdGainsBalHi;
      MODE_ = BAL_HI;
    }
    else if(input=='6') {
      printf("Mode 6\n");
      K_ = params.pdGainsGroundHi;
      MODE_ = GROUND_HI;
    }
  }
}

/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick
void joystickEvents(somatic_d_t& daemon_cx_, Krang::Hardware* krang_,
                    char* b_, double* x_, BalancingConfig& params,
                    double jsFwdAmp_, double jsSpinAmp_, bool& joystickControl_,
                    KRANG_MODE& MODE_, Eigen::Matrix<double, 6, 1>& K_,
                    double& js_forw, double& js_spin) {

  // Change the gains with the given joystick input
  double deltaTH = 0.2, deltaX = 0.02, deltaSpin = 0.02;
  if(!joystickControl_) {
    for(size_t i = 0; i < 4; i++) {
      if(((b_[5] == 0) && (b_[7] == 0)) && (b_[i] == 1)) K_(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
      else if((b_[5] == 1) && (b_[i] == 1)) K_((i % 2) + 2) += ((i < 2) ? deltaX : -deltaX);
      else if((b_[7] == 1) && (b_[i] == 1)) K_((i % 2) + 4) += ((i < 2) ? deltaSpin : -deltaSpin);
    }
  }

  // Update joystick and force-compensation controls
  static int lastb0 = b_[0], lastb1 = b_[1], lastb2 = b_[2];

  if((b_[4] == 1) && (b_[6] == 0) && (b_[0] == 1) && (lastb0 == 0)) {
    joystickControl_ = !joystickControl_;
    if(joystickControl_ == true) {
      somatic_motor_reset(&daemon_cx_, krang_->arms[Krang::LEFT]);
      somatic_motor_reset(&daemon_cx_, krang_->arms[Krang::RIGHT]);
    }
  }

  if((b_[4] == 1) && (b_[6] == 0) && (b_[2] == 1) && (lastb2 == 0)) {
    if(MODE_ == BAL_LO) {
      printf("Mode 5\n");
      K_ = params.pdGainsBalHi;
      MODE_ = BAL_HI;
    }
    else if (MODE_ == BAL_HI) {
      printf("Mode 4\n");
      K_ = params.pdGainsBalLo;
      MODE_ = BAL_LO;
    }
  }

  lastb0 = b_[0], lastb1 = b_[1], lastb2 = b_[2];

  // Ignore the joystick statements for the arm control
  if((b_[4] == 1) || (b_[5] == 1) || (b_[6] == 1) || (b_[7] == 1)) {
    js_forw = js_spin = 0.0;
    return;
  }

  // Set the values for the axis
  if(MODE_ == GROUND_LO || MODE_ == GROUND_HI) {
    js_forw = -params.joystickGainsGroundLo(0) * x_[1];
    js_spin = params.joystickGainsGroundLo(1) * x_[2];
  }
  else if(MODE_ == BAL_LO) {
    js_forw = -params.joystickGainsBalLo(0) * x_[1];
    js_spin = params.joystickGainsBalLo(1) * x_[2];
  }
  else if(MODE_ == BAL_HI) {
    js_forw = -params.joystickGainsBalHi(0) * x_[1];
    js_spin = params.joystickGainsBalHi(1) * x_[2];
  }
  else {
    js_forw = -x_[1] * jsFwdAmp_;
    js_spin = x_[2] * jsSpinAmp_;
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
/// Update Krang Mode based on configuration, state and state error, updates the K matrices used to calculate u/
void updateKrangMode(const Eigen::Matrix<double, 6, 1>& state,
                     const Krang::Hardware* krang_, const BalancingConfig& params,
                     Eigen::Matrix<double, 6, 1>& error,
                     size_t& mode4iter, KRANG_MODE& MODE_,
                     Eigen::Matrix<double, 6, 1>& K_) {
  size_t mode4iterLimit = 100;
  // If in ground Lo mode and waist angle increases beyond 150.0 goto groundHi mode
  if(MODE_ == GROUND_LO) {
    if((krang_->waist->pos[0]-krang_->waist->pos[1])/2.0 < 150.0*M_PI/180.0) {
      MODE_ = GROUND_HI;
      K_ = params.pdGainsGroundHi;
    }
  }
    // If in ground Hi mode and waist angle decreases below 150.0 goto groundLo mode
  else if(MODE_ == GROUND_HI) {
    if((krang_->waist->pos[0]-krang_->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
      MODE_ = GROUND_LO;
      K_ = params.pdGainsGroundLo;
    }
  }

    // If we are in the sit down mode, over write the reference
  else if(MODE_ == SIT) {
    static const double limit = ((-103.0 / 180.0) * M_PI);
    if(krang_->imu < limit) {
      printf("imu (%lf) < limit (%lf): changing to mode 1\n", krang_->imu, limit);
      MODE_ = GROUND_LO;
      K_ = params.pdGainsGroundLo;
    }
    else error(0) = krang_->imu - limit;
  }
    // if in standing up mode check if the balancing angle is reached and stayed, if so switch to balLow mode
  else if(MODE_ == STAND) {
    if(fabs(state(0)) < 0.034) mode4iter++;
    // Change to mode 4 (balance low) if stood up enough
    if(mode4iter > mode4iterLimit) {
      MODE_ = BAL_LO;
      mode4iter = 0;
      K_ = params.pdGainsBalLo;
    }
  }
    // COM error correction in balLow mode
  else if(MODE_ == BAL_LO) {
    // error(0) += 0.005;
  }
    // COM error correction in balHigh mode
  else if(MODE_ == BAL_HI) {
    // error(0) -= 0.005;
  }
}

/* ********************************************************************************************* */
/// Handles the wheel commands if we are started
bool controlStandSit(char* b_, Krang::Hardware* krang_,
                     Eigen::Matrix<double, 6, 1>& state,
                     Eigen::Matrix<double, 6, 1>& error, BalancingConfig& params,
                     KRANG_MODE& MODE_, Eigen::Matrix<double, 6, 1>& K_) {
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
      if((krang_->waist->pos[0] - krang_->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
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
