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
 * @file control.h
 * @author Munzir Zafar
 * @date Oct 31, 2018
 * @brief Header file for control.cpp that implements balancing control functions
 */

#ifndef KRANG_BALANCING_CONTROL_H_
#define KRANG_BALANCING_CONTROL_H_

#include <dart/dart.hpp>
#include <Eigen/Eigen>
#include <kore.hpp>
#include <somatic.h>

/* ******************************************************************************************** */
// Krang Mode Enum
enum KRANG_MODE {
    GROUND_LO,
    STAND,
    SIT,
    BAL_LO,
    BAL_HI,
    GROUND_HI,
    MPC
};

/* ************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center
//  of mass as well
void getState(Krang::Hardware* krang_, dart::dynamics::SkeletonPtr robot_,
              Eigen::Matrix<double, 6, 1>& state, double dt, Eigen::Vector3d* com_);

/* ************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt
//  is last iter. time
void updateReference (double js_forw, double js_spin, double dt,
                      Eigen::Matrix<double, 6, 1>& refState);

/* ************************************************************************** */
/// Handles the wheel commands if we are started
void controlWheels(somatic_d_t& daemon_cx_, bool start_, bool joystickControl_,
                   KRANG_MODE MODE_,  Eigen::Matrix<double, 6, 1>& K_,
                   Eigen::Matrix<double, 6, 1>& error, bool debug,
                   double& lastUleft, double& lastUright,
                   Krang::Hardware* krang_);

/* ************************************************************************** */
// // Change robot's beta values (parameters)
dart::dynamics::SkeletonPtr setParameters(dart::dynamics::SkeletonPtr robot_,
                                          Eigen::MatrixXd betaParams, int bodyParams);

#endif // KRANG_BALANCING_CONTROL_H_
