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
 * @file torso.cpp
 * @author Munzir Zafar
 * @date Oct 30, 2018
 * @brief Implements control of torso based on joystick Input
 */

#include "torso.h"

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic/motor.h>

#include <kore.hpp>
#include <kore/util.hpp>


/* ********************************************************************************************* */
/// Handles the torso commands if we are using joystick
void ControlTorso(somatic_d_t& daemon_cx, TorsoState& torso_state,
                  Krang::Hardware* krang) {

  static TorsoState::TorsoMode last_mode = TorsoState::kStop;

  // if torso needs to be reset
  if(last_mode == TorsoState::kStop && torso_state.mode == TorsoState::kMove) {
    somatic_motor_reset(&daemon_cx, krang->torso);
    last_mode = torso_state.mode;
    return;
  }

  // Control based on the desired state
	if(torso_state.mode == TorsoState::kStop)
    somatic_motor_halt(&daemon_cx, krang->torso);

  else {

	  double dq [] =  {torso_state.command_val};
  	somatic_motor_cmd(&daemon_cx, krang->torso, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY,
                      dq, 1, NULL);
  }
  last_mode = torso_state.mode;
}

