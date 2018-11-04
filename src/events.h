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
 * @brief Header file for events.cpp that maps input/state events to desired control
 * states/modes for the robot
 */

#ifndef KRANG_BALANCING_EVENTS_H_
#define KRANG_BALANCING_EVENTS_H_

#include "arms.h"
#include "balancing_config.h"
#include "control.h"
#include <Eigen/Eigen>
#include "keyboard.h"
#include <kore.hpp>
#include <somatic.h>
#include "torso.h"

void keyboardEvents(kbShared& kb_shared, const BalancingConfig& params, bool& start_,
                    bool& joystickControl_, somatic_d_t& daemon_cx_,
                    Krang::Hardware* krang_, Eigen::Matrix<double, 6, 1>& K_,
                    KRANG_MODE& MODE_);

void joystickBalancingEvents(somatic_d_t& daemon_cx_, Krang::Hardware* krang_,
                             char* b_, double* x_, BalancingConfig& params,
                             bool& joystickControl_,
                             const Eigen::Matrix<double, 6, 1>& state,
                             const Eigen::Matrix<double, 6, 1>& error,
                             KRANG_MODE& MODE_, Eigen::Matrix<double, 6, 1>& K_,
                             double& js_forw, double& js_spin);

void joystickTorsoEvents(const char* b, const double* x, TorsoState* torso_state);

void joyStickArmEvents(const char* b, const double* x, ArmState* arm_state);

Somatic__WaistMode joystickWaistEvents(double x);

bool joystickKillEvent(char* b_);

#endif // KRANG_BALANCING_EVENTS_H_
