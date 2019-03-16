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
#include "balancing/events.h"

#include <somatic.h>
#include <Eigen/Eigen>
#include <kore.hpp>
#include "balancing/arms.h"
#include "balancing/balancing_config.h"
#include "balancing/control.h"
#include "balancing/joystick.h"
#include "balancing/torso.h"

/* ******************************************************************************
 */
/// Events
bool Events(KbShared& kb_shared, Joystick& joystick, bool* start,
            BalanceControl* balance_control, Somatic__WaistMode* waist_mode,
            TorsoState* torso_state, ArmControl* arm_control) {
  KeyboardEvents(kb_shared, start, balance_control, arm_control);
  return JoystickEvents(joystick, balance_control, waist_mode, torso_state,
                        arm_control);
}

/* ********************************************************************************************
 */
// If a character was entered from the keyboard process it

void KeyboardEvents(KbShared& kb_shared, bool* start_,
                    BalanceControl* balance_control, ArmControl* arm_control) {
  char input;

  if (KbCharReceived(kb_shared, &input)) {
    if (input == 's') {
      *start_ = true;
      balance_control->CancelPositionBuiltup();
    }
    // else if(input=='.') readGains();
    else if (input == 'j') {
      arm_control->LockUnlockEvent();
    } else if (input == '1') {
      printf("Mode 1\n");
      balance_control->ForceModeChange(BalanceControl::GROUND_LO);
    } else if (input == '2') {
      printf("Mode 2\n");
      balance_control->ForceModeChange(BalanceControl::STAND);
    } else if (input == '3') {
      printf("Mode 3\n");
      balance_control->ForceModeChange(BalanceControl::SIT);
    } else if (input == '4') {
      printf("Mode 4\n");
      balance_control->ForceModeChange(BalanceControl::BAL_LO);
    } else if (input == '5') {
      printf("Mode 5\n");
      balance_control->ForceModeChange(BalanceControl::BAL_HI);
    } else if (input == '6') {
      printf("Mode 6\n");
      balance_control->ForceModeChange(BalanceControl::GROUND_HI);
    }
  }
}

/* ******************************************************************************
 */
/// Joystick Events
bool JoystickEvents(Joystick& joystick, BalanceControl* balance_control,
                    Somatic__WaistMode* waist_mode, TorsoState* torso_state,
                    ArmControl* arm_control) {
  // Default values
  balance_control->SetFwdInput(0.0);
  balance_control->SetSpinInput(0.0);
  *waist_mode = SOMATIC__WAIST_MODE__STOP;
  arm_control->mode = ArmControl::kStop;
  for (int i = 0; i < 7; i++) arm_control->command_vals[i] = 0.0;
  torso_state->mode = TorsoState::kStop;
  bool kill_program = false;

  // Delta change in gains
  const double deltaTH = 0.2;
  const double deltaX = 0.02;
  const double deltaSpin = 0.02;

  switch (joystick.fingerMode) {
    case (Joystick::L1L2R1R2_FREE): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            case (Joystick::B1_PRESS): {
              balance_control->ChangePdGain(0, +deltaTH);
              break;
            }
            case (Joystick::B2_PRESS): {
              balance_control->ChangePdGain(1, +deltaTH);
              break;
            }
            case (Joystick::B3_PRESS): {
              balance_control->ChangePdGain(0, -deltaTH);
              break;
            }
            case (Joystick::B4_PRESS): {
              balance_control->ChangePdGain(1, -deltaTH);
              break;
            }
            case (Joystick::B10_PRESS): {
              balance_control->StandSitEvent();
              break;
            }
            case (Joystick::RIGHT_THUMB_HORZ_HOLD): {
              double spin = joystick.thumbValue[Joystick::RIGHT];
              balance_control->SetSpinInput(spin);
              break;
            }
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::B9_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              kill_program = true;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::LEFT_THUMB_HORZ_PRESS): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            // case(Joystick::B1_PRESS):
            // case(Joystick::B2_PRESS):
            // case(Joystick::B3_PRESS):
            // case(Joystick::B4_PRESS):
            // case(Joystick::B10_PRESS):
            case (Joystick::RIGHT_THUMB_HORZ_HOLD): {
              double spin = joystick.thumbValue[Joystick::RIGHT];
              balance_control->SetSpinInput(spin);
              break;
            }
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::LEFT_THUMB_VERT_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              double forw = -joystick.thumbValue[Joystick::LEFT];
              balance_control->SetFwdInput(forw);
              break;
            }
            // case(Joystick::B1_PRESS):
            // case(Joystick::B2_PRESS):
            // case(Joystick::B3_PRESS):
            // case(Joystick::B4_PRESS):
            // case(Joystick::B10_PRESS):
            case (Joystick::RIGHT_THUMB_HORZ_HOLD): {
              double forw = -joystick.thumbValue[Joystick::LEFT];
              balance_control->SetFwdInput(forw);
              double spin = joystick.thumbValue[Joystick::RIGHT];
              balance_control->SetSpinInput(spin);
              break;
            }
            case (Joystick::RIGHT_THUMB_VERT_HOLD): {
              double forw = -joystick.thumbValue[Joystick::LEFT];
              balance_control->SetFwdInput(forw);
              break;
            }
          }
          break;
        }
        case (Joystick::CURSOR_HORZ_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              torso_state->mode = TorsoState::kMove;
              double x = joystick.thumbValue[Joystick::LEFT];
              torso_state->command_val = x / 7.0;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::CURSOR_VERT_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              double x = joystick.thumbValue[Joystick::LEFT];
              if (x < -0.9) {
                *waist_mode = SOMATIC__WAIST_MODE__MOVE_FWD;
              } else if (x > 0.9) {
                *waist_mode = SOMATIC__WAIST_MODE__MOVE_REV;
              }
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
      }
      break;
    }
    case (Joystick::L1): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            case (Joystick::B1_PRESS): {
              arm_control->LockUnlockEvent();
              break;
            }
            // case(Joystick::B2_PRESS):
            case (Joystick::B3_PRESS): {
              balance_control->BalHiLoEvent();
              break;
            }
            // case(Joystick::B4_PRESS):
            // case(Joystick::B10_PRESS):
            case (Joystick::RIGHT_THUMB_HORZ_HOLD): {
              arm_control->mode = ArmControl::kMoveLeftBigSet;
              double x = joystick.thumbValue[Joystick::RIGHT];
              arm_control->command_vals[2] = x;
              break;
            }
            case (Joystick::RIGHT_THUMB_VERT_HOLD): {
              arm_control->mode = ArmControl::kMoveLeftBigSet;
              double x = joystick.thumbValue[Joystick::RIGHT];
              arm_control->command_vals[3] = x;
              break;
            }
          }
          break;
        }
        // case(Joystick::B9_PRESS):
        case (Joystick::LEFT_THUMB_HORZ_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveLeftBigSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[0] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::LEFT_THUMB_VERT_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveLeftBigSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[1] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
          // case(Joystick::CURSOR_HORZ_PRESS):
          // case(Joystick::CURSOR_VERT_PRESS):
      }
      break;
    }
    case (Joystick::L2): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            // case(Joystick::B1_PRESS):
            // case(Joystick::B2_PRESS):
            // case(Joystick::B3_PRESS):
            // case(Joystick::B4_PRESS):
            // case(Joystick::B10_PRESS):
            case (Joystick::RIGHT_THUMB_HORZ_HOLD): {
              arm_control->mode = ArmControl::kMoveLeftSmallSet;
              double x = joystick.thumbValue[Joystick::RIGHT];
              arm_control->command_vals[6] = x;
              break;
            }
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        // case(Joystick::B9_PRESS):
        case (Joystick::LEFT_THUMB_HORZ_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveLeftSmallSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[4] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::LEFT_THUMB_VERT_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveLeftSmallSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[5] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
          // case(Joystick::CURSOR_HORZ_PRESS):
          // case(Joystick::CURSOR_VERT_PRESS):
      }
      break;
    }
    case (Joystick::R1): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            case (Joystick::B1_PRESS): {
              balance_control->ChangePdGain(2, +deltaX);
              break;
            }
            case (Joystick::B2_PRESS): {
              balance_control->ChangePdGain(3, +deltaX);
              break;
            }
            case (Joystick::B3_PRESS): {
              balance_control->ChangePdGain(2, -deltaX);
              break;
            }
            case (Joystick::B4_PRESS): {
              balance_control->ChangePdGain(3, -deltaX);
              break;
            }
            // case(Joystick::B10_PRESS):
            case (Joystick::RIGHT_THUMB_HORZ_HOLD): {
              arm_control->mode = ArmControl::kMoveRightBigSet;
              double x = joystick.thumbValue[Joystick::RIGHT];
              arm_control->command_vals[2] = x;
              break;
            }
            case (Joystick::RIGHT_THUMB_VERT_HOLD): {
              arm_control->mode = ArmControl::kMoveRightBigSet;
              double x = joystick.thumbValue[Joystick::RIGHT];
              arm_control->command_vals[3] = x;
              break;
            }
          }
          break;
        }
        // case(Joystick::B9_PRESS):
        case (Joystick::LEFT_THUMB_HORZ_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveRightBigSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[0] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::LEFT_THUMB_VERT_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveRightBigSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[1] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
          // case(Joystick::CURSOR_HORZ_PRESS):
          // case(Joystick::CURSOR_VERT_PRESS):
      }
      break;
    }
    case (Joystick::R2): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            case (Joystick::B1_PRESS): {
              balance_control->ChangePdGain(4, +deltaSpin);
              break;
            }
            case (Joystick::B2_PRESS): {
              balance_control->ChangePdGain(5, +deltaSpin);
              break;
            }
            case (Joystick::B3_PRESS): {
              balance_control->ChangePdGain(4, -deltaSpin);
              break;
            }
            case (Joystick::B4_PRESS): {
              balance_control->ChangePdGain(5, -deltaSpin);
              break;
            }
            // case(Joystick::B10_PRESS):
            case (Joystick::RIGHT_THUMB_HORZ_HOLD): {
              arm_control->mode = ArmControl::kMoveRightSmallSet;
              double x = joystick.thumbValue[Joystick::RIGHT];
              arm_control->command_vals[6] = x;
              break;
            }
              // case(Joystick::RIGHT_THUMB_VERT_HOLD):
          }
          break;
        }
        // case(Joystick::B9_PRESS):
        case (Joystick::LEFT_THUMB_HORZ_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveRightSmallSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[4] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
        case (Joystick::LEFT_THUMB_VERT_HOLD): {
          switch (joystick.rightMode) {
            case (Joystick::RIGHT_THUMB_FREE): {
              arm_control->mode = ArmControl::kMoveRightSmallSet;
              double x = joystick.thumbValue[Joystick::LEFT];
              arm_control->command_vals[5] = x;
              break;
            }
              // case(Joystick::B1_PRESS):
              // case(Joystick::B2_PRESS):
              // case(Joystick::B3_PRESS):
              // case(Joystick::B4_PRESS):
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
          // case(Joystick::CURSOR_HORZ_PRESS):
          // case(Joystick::CURSOR_VERT_PRESS):
      }
      break;
    }
    case (Joystick::L1L2): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            case (Joystick::B1_HOLD): {
              arm_control->mode = ArmControl::kMoveLeftToPresetPos;
              arm_control->preset_config_num = 1;
              break;
            }
            case (Joystick::B2_HOLD): {
              arm_control->mode = ArmControl::kMoveLeftToPresetPos;
              arm_control->preset_config_num = 2;
              break;
            }
            case (Joystick::B3_HOLD): {
              arm_control->mode = ArmControl::kMoveLeftToPresetPos;
              arm_control->preset_config_num = 3;
              break;
            }
            case (Joystick::B4_HOLD): {
              arm_control->mode = ArmControl::kMoveLeftToPresetPos;
              arm_control->preset_config_num = 4;
              break;
            }
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
          // case(Joystick::B9_PRESS):
          // case(Joystick::LEFT_THUMB_HORZ_PRESS):
          // case(Joystick::LEFT_THUMB_VERT_PRESS):
          // case(Joystick::CURSOR_HORZ_PRESS):
          // case(Joystick::CURSOR_VERT_PRESS):
      }
      break;
    }
    case (Joystick::R1R2): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            case (Joystick::B1_HOLD): {
              arm_control->mode = ArmControl::kMoveRightToPresetPos;
              arm_control->preset_config_num = 1;
              break;
            }
            case (Joystick::B2_HOLD): {
              arm_control->mode = ArmControl::kMoveRightToPresetPos;
              arm_control->preset_config_num = 2;
              break;
            }
            case (Joystick::B3_HOLD): {
              arm_control->mode = ArmControl::kMoveRightToPresetPos;
              arm_control->preset_config_num = 3;
              break;
            }
            case (Joystick::B4_HOLD): {
              arm_control->mode = ArmControl::kMoveRightToPresetPos;
              arm_control->preset_config_num = 4;
              break;
            }
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
          // case(Joystick::B9_PRESS):
          // case(Joystick::LEFT_THUMB_HORZ_PRESS):
          // case(Joystick::LEFT_THUMB_VERT_PRESS):
          // case(Joystick::CURSOR_HORZ_PRESS):
          // case(Joystick::CURSOR_VERT_PRESS):
      }
      break;
    }
    // case(Joystick::L1R1):
    // case(Joystick::L1R2):
    // case(Joystick::L2R1):
    // case(Joystick::L2R2):
    // case(Joystick::L1L2R1):
    // case(Joystick::L1L2R2):
    // case(Joystick::L1R1R2):
    // case(Joystick::L2R1R2):
    case (Joystick::L1L2R1R2): {
      switch (joystick.leftMode) {
        case (Joystick::LEFT_THUMB_FREE): {
          switch (joystick.rightMode) {
            // case(Joystick::RIGHT_THUMB_FREE):
            case (Joystick::B1_HOLD): {
              arm_control->mode = ArmControl::kMoveBothToPresetPos;
              arm_control->preset_config_num = 1;
              break;
            }
            case (Joystick::B2_HOLD): {
              arm_control->mode = ArmControl::kMoveBothToPresetPos;
              arm_control->preset_config_num = 1;
              break;
            }
            case (Joystick::B3_HOLD): {
              arm_control->mode = ArmControl::kMoveBothToPresetPos;
              arm_control->preset_config_num = 1;
              break;
            }
            case (Joystick::B4_HOLD): {
              arm_control->mode = ArmControl::kMoveBothToPresetPos;
              arm_control->preset_config_num = 1;
              break;
            }
              // case(Joystick::B10_PRESS):
              // case(Joystick::RIGHT_THUMB_HORZ_PRESS):
              // case(Joystick::RIGHT_THUMB_VERT_PRESS):
          }
          break;
        }
          // case(Joystick::B9_PRESS):
          // case(Joystick::LEFT_THUMB_HORZ_PRESS):
          // case(Joystick::LEFT_THUMB_VERT_PRESS):
          // case(Joystick::CURSOR_HORZ_PRESS):
          // case(Joystick::CURSOR_VERT_PRESS):
      }
      break;
    }
  }

  return kill_program;
}
