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

void keyboardEvents(kbShared& kb_shared, bool& start_,
                    BalanceControl& balance_control, ArmControl& arm_control) {

  char input;

  if(kbCharReceived(kb_shared, &input)) {

    if(input=='s') {
      start_ = true;
      balance_control.CancelPositionBuiltup();
    }
    //else if(input=='.') readGains();
    else if(input=='j') {
      arm_control.LockUnlockEvent();
      std::cout << "[INFO] Keyboard based lock unlock event called" << std::endl;
    }
    else if(input=='1') {
      printf("Mode 1\n");
      balance_control.ForceModeChange(BalanceControl::GROUND_LO);
    }
    else if(input=='2') {
      printf("Mode 2\n");
      balance_control.ForceModeChange(BalanceControl::STAND);
    }
    else if(input=='3') {
      printf("Mode 3\n");
      balance_control.ForceModeChange(BalanceControl::SIT);
    }
    else if(input=='4') {
      printf("Mode 4\n");
      balance_control.ForceModeChange(BalanceControl::BAL_LO);
    }
    else if(input=='5') {
      printf("Mode 5\n");
      balance_control.ForceModeChange(BalanceControl::BAL_HI);
    }
    else if(input=='6') {
      printf("Mode 6\n");
      balance_control.ForceModeChange(BalanceControl::GROUND_HI);
    }
  }
}

/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick
void joystickBalancingEvents(char* b_, double* x_,
                             BalanceControl& balance_control) {

  // Change the gains with the given joystick input
  const double deltaTH = 0.2;
  const double deltaX = 0.02;
  const double deltaSpin = 0.02;
  if(b_[5] == 0 && b_[7] == 0) {
    if     (b_[0] == 1) balance_control.ChangePdGain(0, +deltaTH);
    else if(b_[1] == 1) balance_control.ChangePdGain(1, +deltaTH);
    else if(b_[2] == 1) balance_control.ChangePdGain(0, -deltaTH);
    else if(b_[3] == 1) balance_control.ChangePdGain(1, -deltaTH);
  }
  else if(b_[5] == 1 && b_[7] == 0) {
    if     (b_[0] == 1) balance_control.ChangePdGain(2, +deltaX);
    else if(b_[1] == 1) balance_control.ChangePdGain(3, +deltaX);
    else if(b_[2] == 1) balance_control.ChangePdGain(2, -deltaX);
    else if(b_[3] == 1) balance_control.ChangePdGain(3, -deltaX);
  }
  else if(b_[5] == 0 && b_[7] == 1) {
    if     (b_[0] == 1) balance_control.ChangePdGain(4, +deltaSpin);
    else if(b_[1] == 1) balance_control.ChangePdGain(5, +deltaSpin);
    else if(b_[2] == 1) balance_control.ChangePdGain(4, -deltaSpin);
    else if(b_[3] == 1) balance_control.ChangePdGain(5, -deltaSpin);
  }

  // Gain Change
  static int lastb2 = b_[2];
  if((b_[4] == 1) && (b_[6] == 0) && (b_[2] == 1) && (lastb2 == 0)) {
    balance_control.BalHiLoEvent();
  }
  lastb2 = b_[2];

  // Ignore the joystick statements for the arm control
  if((b_[4] == 1) || (b_[5] == 1) || (b_[6] == 1) || (b_[7] == 1)) {
    balance_control.SetFwdSpinInputs(0.0, 0.0);
  } else {
    balance_control.SetFwdSpinInputs(-x_[1], x_[2]);
  }

  // Stand/Sit if button 10 is pressed and conditions are right
  static bool b9Prev = 0;
  if(b9Prev == 0 && b_[9] == 1) {
    balance_control.StandSitEvent();
  }
  b9Prev = b_[9];
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
void joyStickArmEvents(const char* b, const double* x, ArmControl* arm_control) {

  // Lock or unlock arms
  static int lastb0 = b[0];
  if((b[4] == 1) && (b[6] == 0) && (b[0] == 1) && (lastb0 == 0)) {
    arm_control->LockUnlockEvent();
    std::cout << "[INFO] Joystick based lock unlock event called" << std::endl;
  }
  lastb0 = b[0];

  // Go To Preset Positions
  if(((b[4] == 1) && (b[6] == 1)) || ((b[5] == 1) && (b[7] == 1))) {

    bool noConfs = true;
    for(size_t i = 0; i < 4; i++) {
      if(b[i] == 1) {
        if((b[4] == 1) && (b[6] == 1) && (b[5] == 1) && (b[7] == 1)) {
          arm_control->mode = ArmControl::kMoveBothToPresetPos;
          arm_control->preset_config_num = i;
        }
        else if((b[4] == 1) && (b[6] == 1)) {
          arm_control->mode = ArmControl::kMoveLeftToPresetPos;
          arm_control->preset_config_num = i;
        }
        else if((b[5] == 1) && (b[7] == 1))  {
          arm_control->mode = ArmControl::kMoveRightToPresetPos;
          arm_control->preset_config_num = i;
        }
        noConfs = false;
      }
    }

    // If nothing is pressed, stop the arms
    if(noConfs) {
      arm_control->mode = ArmControl::kStop;
    }
  }
  // If only one of the front buttons is pressed
  else {
    if(b[4] && !b[6]) {
      arm_control->mode = ArmControl::kMoveLeftBigSet;
      for(int i = 0; i < 4; i++)
        arm_control->command_vals[i] = x[i];
    } else if(!b[4] && b[6]) {
      arm_control->mode = ArmControl::kMoveLeftSmallSet;
      for(int i = 4; i < 7; i++)
        arm_control->command_vals[i] = x[i-4];
    } else if(b[5] && !b[7]) {
      arm_control->mode = ArmControl::kMoveRightBigSet;
      for(int i = 0; i < 4; i++)
        arm_control->command_vals[i] = x[i];
    } else if(!b[5] && b[7]) {
      arm_control->mode = ArmControl::kMoveRightSmallSet;
      for(int i = 4; i < 7; i++)
        arm_control->command_vals[i] = x[i-4];
    }
    else {
      arm_control->mode = ArmControl::kStop;
    }
  }
}

/* **************************************************************************** */
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

/* ****************************************************************************** */
/// Kill Event
bool joystickKillEvent(char* b_) {
  // ==========================================================================
  // Quit if button 9 on the joystick is pressed

  if(b_[8] == 1) return false;
  return true;
}
