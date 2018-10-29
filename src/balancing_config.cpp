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
 * @file balancing_config.cpp
 * @author Munzir Zafar
 * @date Oct 29, 2018
 * @brief Reads configuration parameters from a cfg file
 */

#include "balancing_config.h"

#include <assert.h>
#include <iostream>
#include <memory>

#include <config4cpp/Configuration.h>
#include <Eigen/Eigen>

// Function for reading configuration parameters. Location to cfg file is needed at the input.
// Output is the pointer to the struct that contains all the params.
// Ownership to the allocated memory is unique and transfered to the caller function
std::unique_ptr<BalancingConfig> ReadConfigParams(const char* config_file) {

  // Allocate memory for the output of the function
  std::unique_ptr<BalancingConfig> params (new BalancingConfig);

  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << "Reading configuration parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    // Read the Q matrix for LQR
    params->lqrQ.setZero();
    str = cfg->lookupString(scope, "lqrQ");
    stream.str(str);
    for(int i = 0; i < 4; i++)
      stream >> params->lqrQ(i, i);
    stream.clear();
    std::cout << "lqrQ: " << params->lqrQ << std::endl;

    // Read the R matrix for LQR
    params->lqrR.setZero();
    str = cfg->lookupString(scope, "lqrR");
    stream.str(str);
    for(int i = 0; i < 1; i++)
      stream >> params->lqrR(i, i);
    stream.clear();
    std::cout << "lqrR: " << params->lqrR << std::endl;

    // Read PD Gains
    const char* pdGainsStrings[] = {
      "pdGainsGroundLo", "pdGainsGroundHi",
      "pdGainsStand", "pdGainsSit",
      "pdGainsBalLo", "pdGainsBalHi"
    };
    Eigen::Matrix<double, 6, 1>* pdGains[] = {
      &params->pdGainsGroundLo, &params->pdGainsGroundHi,
      &params->pdGainsStand, &params->pdGainsSit,
      &params->pdGainsBalLo, &params->pdGainsBalHi
    };
    for(int i = 0; i<6; i++) {
      str = cfg->lookupString(scope, pdGainsStrings[i]);
      stream.str(str);
      for(int j = 0; j < 6; j++)
        stream >> (*(pdGains[i]))(j);
      stream.clear();
      std::cout << pdGainsStrings[i] << ": ";
      std::cout << (*(pdGains[i])).transpose() << std::endl;
    }

    // Read Joystick Gains
    const char* joystickGainsStrings[] = {
      "joystickGainsGroundLo", "joystickGainsGroundHi",
      "joystickGainsStand", "joystickGainsSit",
      "joystickGainsBalLo", "joystickGainsBalHi"
    };
    Eigen::Matrix<double, 2, 1>* joystickGains[] = {
      &params->joystickGainsGroundLo, &params->joystickGainsGroundHi,
      &params->joystickGainsStand, &params->joystickGainsSit,
      &params->joystickGainsBalLo, &params->joystickGainsBalHi
    };
    for(int i = 0; i<6; i++) {
      str = cfg->lookupString(scope, joystickGainsStrings[i]);
      stream.str(str);
      for(int j = 0; j < 2; j++)
        stream >> (*(joystickGains[i]))(j);
      stream.clear();
      std::cout << joystickGainsStrings[i] << ": ";
      std::cout << (*(joystickGains[i])).transpose() << std::endl;
    }
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading config parameters");
  }
  std::cout << std::endl;

  return params;
}

