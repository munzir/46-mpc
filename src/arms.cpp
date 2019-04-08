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

#include "balancing/arms.h"

#include <config4cpp/Configuration.h>
#include <math.h>
#include <unistd.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic/motor.h>

#include <kore.hpp>
#include <kore/util.hpp>

#include "balancing/balancing_config.h"

/* ************************************************************************************/
// Read preset arm configurations from config file
void ArmControl::ReadPresetConfig(const char* config_file,
                                  double output_array[8][7]) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << std::endl
            << "Reading arms configuration parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    // Read preset arm configurations
    const char* presetArmStrings[] = {
        "left_preset_1", "right_preset_1", "left_preset_2", "right_preset_2",
        "left_preset_3", "right_preset_3", "left_preset_4", "right_preset_4"};
    for (int i = 0; i < 8; i++) {
      str = cfg->lookupString(scope, presetArmStrings[i]);
      stream.str(str);
      for (int j = 0; j < 7; j++) stream >> output_array[i][j];
      stream.clear();
      std::cout << presetArmStrings[i] << ":";
      for (int j = 0; j < 7; j++) std::cout << " " << output_array[i][j];
      std::cout << std::endl;
    }
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading arm config parameters");
  }
}

/* ************************************************************************************/
/// Constructor
ArmControl::ArmControl(somatic_d_t* daemon_cx_, Krang::Hardware* krang_,
                       BalancingConfig& params)
    : krang(krang_), daemon_cx(daemon_cx_) {
  ReadPresetConfig("/usr/local/share/krang/balancing/cfg/arms_params.cfg",
                   presetArmConfs);
  event_based_lock_unlock = params.manualArmLockUnlock;
  somatic_motor_halt(daemon_cx, krang->arms[Krang::LEFT]);
  somatic_motor_halt(daemon_cx, krang->arms[Krang::RIGHT]);
  usleep(1e5);
  halted = true;
  mode = kStop;
}

/* ************************************************************************************/
// If event_based_lock_unlock is not active, unlock the arm if it just began to
// be used Returns true if a reset was performed
bool ArmControl::ArmResetIfNeeded(ArmControl::ArmMode& last_mode) {
  if (!event_based_lock_unlock) {
    // If left arm was on break and needs to be reset
    if ((last_mode == ArmControl::kStop ||
         last_mode == ArmControl::kMoveRightBigSet ||
         last_mode == ArmControl::kMoveRightSmallSet ||
         last_mode == ArmControl::kMoveRightToPresetPos) &&
        (mode == ArmControl::kMoveLeftBigSet ||
         mode == ArmControl::kMoveLeftSmallSet ||
         mode == ArmControl::kMoveLeftToPresetPos ||
         mode == ArmControl::kMoveBothToPresetPos)) {
      somatic_motor_reset(daemon_cx, krang->arms[Krang::LEFT]);

      // return to allow delay after reset (assuming that by the time this
      // function is called again, some time will have passed)
      last_mode = mode;
      std::cout << "[INFO] ArmResetIfNeeded just reset the left arm"
                << std::endl;
      return true;
    }

    // If right arm needs to be reset
    else if ((last_mode == ArmControl::kStop ||
              last_mode == ArmControl::kMoveLeftBigSet ||
              last_mode == ArmControl::kMoveLeftSmallSet ||
              last_mode == ArmControl::kMoveLeftToPresetPos) &&
             (mode == ArmControl::kMoveRightBigSet ||
              mode == ArmControl::kMoveRightSmallSet ||
              mode == ArmControl::kMoveRightToPresetPos ||
              mode == ArmControl::kMoveBothToPresetPos)) {
      somatic_motor_reset(daemon_cx, krang->arms[Krang::RIGHT]);

      // return to allow delay after reset
      last_mode = mode;
      std::cout << "[INFO] ArmResetIfNeeded just reset the right arm"
                << std::endl;
      return true;
    }
  }
  return false;
}

/* ************************************************************************************/
/// Stop left and right arms. Either halt or zero speed is commanded based on
/// event_based_lock_unlock flag
void ArmControl::StopLeftArm() {
  if (!event_based_lock_unlock) {
    somatic_motor_halt(daemon_cx, krang->arms[Krang::LEFT]);
  } else {
    double dq[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    somatic_motor_cmd(daemon_cx, krang->arms[Krang::LEFT],
                      SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
  }
}
void ArmControl::StopRightArm() {
  if (!event_based_lock_unlock) {
    somatic_motor_halt(daemon_cx, krang->arms[Krang::RIGHT]);
  } else {
    double dq[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    somatic_motor_cmd(daemon_cx, krang->arms[Krang::RIGHT],
                      SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
  }
}
/* ************************************************************************************/
void ArmControl::ArmLockEvent() {
  if (event_based_lock_unlock) {
    somatic_motor_halt(daemon_cx, krang->arms[Krang::LEFT]);
    somatic_motor_halt(daemon_cx, krang->arms[Krang::RIGHT]);
  }
}
void ArmControl::ArmUnlockEvent() {
  if (event_based_lock_unlock) {
    somatic_motor_reset(daemon_cx, krang->arms[Krang::LEFT]);
    somatic_motor_reset(daemon_cx, krang->arms[Krang::RIGHT]);
  }
}
void ArmControl::LockUnlockEvent() {
  if (halted) {
    ArmUnlockEvent();
    halted = false;
  } else {
    ArmLockEvent();
    halted = true;
  }
}

/* ************************************************************************************/
/// Controls the arms
void ArmControl::ControlArms() {
  // No control command should be sent to the motors when they are locked
  if (event_based_lock_unlock && halted) return;

  static ArmControl::ArmMode last_mode = ArmControl::kStop;

  // If event_based_lock_unlock is not active, unlock the arm if it just
  // began to be used
  // Return if a reset was performed, this allows delay till next iteration to
  // let hardware unlocking to complete
  if (ArmResetIfNeeded(last_mode)) return;

  // Control the arm based on the desired arm state
  switch (mode) {
    case ArmControl::kStop: {
      // Halt both arms
      StopLeftArm();
      StopRightArm();
      break;
    }
    case ArmControl::kMoveLeftBigSet: {
      // Halt the right arm
      StopRightArm();

      // Send commanded vels to big motors of left arm and zero vels to other
      // motors
      double dq[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for (int i = 0; i < 4; i++) dq[i] = command_vals[i];
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmControl::kMoveLeftSmallSet: {
      // Halt the right arm
      StopRightArm();

      // Send commanded vels to small motors of left arm and zero vels to other
      // motors
      double dq[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for (int i = 4; i < 7; i++) dq[i] = command_vals[i];
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmControl::kMoveRightBigSet: {
      // Halt left arm
      StopLeftArm();

      // Send commanded vels to big motors of the right arm and zero vels to
      // others
      double dq[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for (int i = 0; i < 4; i++) dq[i] = command_vals[i];
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmControl::kMoveRightSmallSet: {
      // Halt left arm
      StopLeftArm();

      // Send commanded vels to small motors of the right arm and zero vels to
      // others
      double dq[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for (int i = 4; i < 7; i++) dq[i] = command_vals[i];
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
      break;
    }
    case ArmControl::kMoveLeftToPresetPos: {
      // Halt the right arm
      StopRightArm();

      // Send preset config positions to left arm
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2 * (preset_config_num-1)], 7, NULL);
      break;
    }
    case ArmControl::kMoveRightToPresetPos: {
      // Halt left arm
      StopLeftArm();

      // Send preset config position to the right arm
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2 * (preset_config_num-1) + 1], 7, NULL);
      break;
    }
    case ArmControl::kMoveBothToPresetPos: {
      // Send present config positions to both arms
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2 * (preset_config_num-1)], 7, NULL);
      somatic_motor_cmd(daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                        presetArmConfs[2 * (preset_config_num-1) + 1], 7, NULL);
      break;
    }
    default: {
      std::cout << "bad arm state " << std::endl;
      assert(false && "Arm Mode has invalid value");
    }
  }

  last_mode = mode;
}
