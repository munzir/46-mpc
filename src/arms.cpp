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
 * @file arms.cpp
 * @author Munzir Zafar
 * @date Oct 29, 2018
 * @brief Implements control of arms based on joystick Input
 */

#include "arms.h"

#include <math.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic/motor.h>

#include <kore.hpp>
#include <kore/util.hpp>


/* ************************************************************************************/
/// Controls the arms
void controlArms(somatic_d_t& daemon_cx, const ArmState& arm_state, Krang::Hardware* krang) {

  static ArmState::ArmMode last_mode = ArmState::kStop;

  // If left arm was on break and needs to be reset
  if((last_mode == ArmState::kStop ||
      last_mode == ArmState::kMoveRightBigSet ||
      last_mode == ArmState::kMoveRightSmallSet ||
      last_mode == ArmState::kMoveRightToPresetPos) &&
     (arm_state.mode == ArmState::kMoveLeftBigSet ||
      arm_state.mode == ArmState::kMoveLeftSmallSet ||
      arm_state.mode == ArmState::kMoveLeftToPresetPos ||
      arm_state.mode == ArmState::kMoveBothToPresetPos)) {

    somatic_motor_reset(&daemon_cx, krang->arms[Krang::LEFT]);

    // return to allow delay after reset (assuming that by the time this function is called
    // again, some time will have passed)
    last_mode = arm_state.mode;
    return;
  }

  // If right arm needs to be reset
  if((last_mode == ArmState::kStop ||
      last_mode == ArmState::kMoveLeftBigSet ||
      last_mode == ArmState::kMoveLeftSmallSet ||
      last_mode == ArmState::kMoveLeftToPresetPos) &&
     (arm_state.mode == ArmState::kMoveRightBigSet ||
      arm_state.mode == ArmState::kMoveRightSmallSet ||
      arm_state.mode == ArmState::kMoveRightToPresetPos ||
      arm_state.mode == ArmState::kMoveBothToPresetPos)) {

    somatic_motor_reset(&daemon_cx, krang->arms[Krang::RIGHT]);

    // return to allow delay after reset
    last_mode = arm_state.mode;
    return;
  }


  // Control the arm based on the desired arm state
  switch (arm_state.mode) {
    case ArmState::kStop: {
			// Halt both arms
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::LEFT]);
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::RIGHT]);
      break;
    }
    case ArmState::kMoveLeftBigSet: {
      // Halt the right arm
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::RIGHT]);

      // Send commanded vels to big motors of left arm and zero vels to other motors
      double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for(int i = 0; i < 4; i++)
        dq[i] = arm_state.command_vals[i];
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmState::kMoveLeftSmallSet: {
      // Halt the right arm
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::RIGHT]);

      // Send commanded vels to small motors of left arm and zero vels to other motors
      double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for(int i = 4; i < 7; i++)
        dq[i] = arm_state.command_vals[i];
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmState::kMoveRightBigSet: {
			// Halt left arm
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::LEFT]);

      // Send commanded vels to big motors of the right arm and zero vels to others
      double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for(int i = 0; i < 4; i++)
        dq[i] = arm_state.command_vals[i];
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmState::kMoveRightSmallSet: {
			// Halt left arm
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::LEFT]);

      // Send commanded vels to small motors of the right arm and zero vels to others
      double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for(int i = 4; i < 7; i++)
        dq[i] = arm_state.command_vals[i];
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmState::kMoveLeftToPresetPos: {
      // Halt the right arm
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::RIGHT]);

      // Send preset config positions to left arm
      somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2*arm_state.preset_config_num],
                        7, NULL);
      break;
    }
    case ArmState::kMoveRightToPresetPos: {
			// Halt left arm
			somatic_motor_halt(&daemon_cx, krang->arms[Krang::LEFT]);

      // Send preset config position to the right arm
      somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2*arm_state.preset_config_num+1],
                        7, NULL);
      break;
    }
    case ArmState::kMoveBothToPresetPos: {
      // Send present config positions to both arms
      somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2*arm_state.preset_config_num],
                        7, NULL);
      somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2*arm_state.preset_config_num+1],
                        7, NULL);
      break;
    }
    default: {
      std::cout << "bad arm state " << std::endl;
      assert(false && "Arm Mode has invalid value");
    }
  }

  last_mode = arm_state.mode;
}
