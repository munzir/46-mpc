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
 * @file arms.h
 * @author Munzir Zafar
 * @date Oct 29, 2018
 * @brief Header for arms.cpp that controls arms in the balancing code based on
 * input
 */

#ifndef KRANG_BALANCING_ARMS_H_
#define KRANG_BALANCING_ARMS_H_

#include <somatic.h>
#include <somatic/daemon.h>
#include <kore.hpp>
#include "balancing_config.h"

/* *********************************************************************************************
 */
// The struct used by controlArm function to decide the control command to be
// sent
class ArmControl {
 public:
  enum ArmMode {
    kStop,               // both arms' halted
    kMoveLeftBigSet,     // sets left arms' joints 1-4 vel to command_vals[0-3]
    kMoveLeftSmallSet,   // sets left arms' joints 5-7 vel to command_vals[4-6]
    kMoveRightBigSet,    //  sets left arms' joints 1-4 vel to command_vals[0-3]
    kMoveRightSmallSet,  //  sets left arms' joints 5-7 vel to command_vals[4-6]
    kMoveLeftToPresetPos,   // left arms' pos =
                            // presetArmConfs[2*preset_config_num]
    kMoveRightToPresetPos,  // right arms' pos =
                            // presetArmConfs[2*preset_config_num+1]
    kMoveBothToPresetPos,    // does both the above
    kMoveBothAtRefSpeed  // Move both arms at desired speeds set in
                         // command_vals[0-6] for left arm and
                         // more_command_vals[0-6] for the right arm
  };
  double presetArmConfs[8][7];  // should be const but somatic_motor_cmd
                                // gives problems when passing directly
                                // const array pointers to it

  ArmControl(somatic_d_t* daemon_cx_, Krang::Hardware* krang_,
             BalancingConfig& params);
  ~ArmControl(){};

  void ControlArms();
  void LockUnlockEvent();

  ArmMode mode;
  int preset_config_num;
  double command_vals[7];
  double more_command_vals[7];

 private:
  void ReadPresetConfig(const char* config_file, double output_array[8][7]);
  bool ArmResetIfNeeded(ArmMode& last_mode);
  void StopLeftArm();
  void StopRightArm();

  somatic_d_t* daemon_cx;
  Krang::Hardware* krang;
  bool event_based_lock_unlock;
  bool halted;  // to manage halt/reset events when state_based_lock_unlock is
                // not set
  void ArmLockEvent();
  void ArmUnlockEvent();
};

#endif  // KRANG_BALANCING_ARMS_H_
