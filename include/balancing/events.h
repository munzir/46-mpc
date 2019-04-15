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
 * @file events.h
 * @author Munzir Zafar
 * @date Oct 31, 2018
 * @brief Header file for events.cpp that maps input/state events to desired
 * control states/modes for the robot
 */

#ifndef KRANG_BALANCING_EVENTS_H_
#define KRANG_BALANCING_EVENTS_H_

#include <somatic.h>
#include <Eigen/Eigen>
#include <kore.hpp>
#include "arms.h"
#include "balancing_config.h"
#include "control.h"
#include "joystick.h"
#include "keyboard.h"
#include "torso.h"

/* ******************************************************************************
 */
/// Events
bool Events(KbShared& kb_shared, Joystick& joystick, bool* start,
            BalanceControl* balance_control, Somatic__WaistMode* waist_mode,
            TorsoState* torso_state, ArmControl* arm_control,
            bool* log_mark = NULL);

/* ********************************************************************************************
 */
// If a character was entered from the keyboard process it
void KeyboardEvents(KbShared& kb_shared, bool* start_,
                    BalanceControl* balance_control, ArmControl* arm_control);

/* ******************************************************************************
 */
/// Joystick Events
bool JoystickEvents(Joystick& joystick, BalanceControl* balance_control,
                    Somatic__WaistMode* waist_mode, TorsoState* torso_state,
                    ArmControl* arm_control, bool* log_mark = NULL);

#endif  // KRANG_BALANCING_EVENTS_H_
