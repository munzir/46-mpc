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
 * @file balancing_config.h
 * @author Munzir Zafar
 * @date Oct 29, 2018
 * @brief Header for balancing_config.cpp that reads configuration parameters
 * from a cfg file
 */

#ifndef KRANG_BALANCING_CONFIG_H_
#define KRANG_BALANCING_CONFIG_H_

#include <memory>
#include <Eigen/Eigen>

// Structure in which all configurable parameters are read at the beginning of
// the program
struct BalancingConfig {

  // Path to urdf file
  char urdfpath[1024];

  // Path to CoM estimation model parameters
  char comParametersPath[1024];

  // PD Gains
  Eigen::Matrix<double, 6, 1> pdGainsGroundLo;
  Eigen::Matrix<double, 6, 1> pdGainsGroundHi;
  Eigen::Matrix<double, 6, 1> pdGainsStand;
  Eigen::Matrix<double, 6, 1> pdGainsSit;
  Eigen::Matrix<double, 6, 1> pdGainsBalLo;
  Eigen::Matrix<double, 6, 1> pdGainsBalHi;

  // Joystick Gains
  double joystickGainsGroundLo[2];
  double joystickGainsGroundHi[2];
  double joystickGainsStand[2];
  double joystickGainsSit[2];
  double joystickGainsBalLo[2];
  double joystickGainsBalHi[2];

  // Q and R matrices for LQR
  bool dynamicLQR;
  Eigen::Matrix<double, 4, 4> lqrQ;
  Eigen::Matrix<double, 1, 1> lqrR;

  // Balancing control mode transition parameters
  double imuSitAngle;
  double toBalThreshold;

  // if this flag is not set, arms are always locked when not in use
  // if this flag is set, arms have to be unhalted by a joystick-based event
  // before use, and have to be halted in order to lock them
  bool manualArmLockUnlock;
};

// Function for reading configuration parameters. First argument is the location of
// cfg file from the parameters are to be read. Second argument is the output where
// the parameters are stored
void ReadConfigParams (const char* config_file, BalancingConfig* params);

#endif // KRANG_BALANCING_CONFIG_H_
