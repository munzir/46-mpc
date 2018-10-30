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
void controlArms(somatic_d_t& daemon_cx, const char* b, const double* x,
                 Krang::Hardware* krang) {

	// Return if the x[3] is being used for robotiq hands
	if(fabs(x[3]) > 0.2) return;

	// Check if one of the preset configurations are requested by pressing 9 and
	// any of the buttons from 1 to 4 at the same time
	if(((b[4] == 1) && (b[6] == 1)) || ((b[5] == 1) && (b[7] == 1))) {

		// Check if the button is pressed for the arm configuration is pressed, if so send pos commands
		bool noConfs = true;
		for(size_t i = 0; i < 4; i++) {
			if(b[i] == 1) {
				if((b[4] == 1) && (b[6] == 1))
					somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT],
                            SOMATIC__MOTOR_PARAM__MOTOR_POSITION, presetArmConfs[2*i],
                            7, NULL);
				if((b[5] == 1) && (b[7] == 1))  {
					somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT],
                            SOMATIC__MOTOR_PARAM__MOTOR_POSITION, presetArmConfs[2*i+1],
                            7, NULL);
				}
				noConfs = false;
				return;
			}
		}

		// If nothing is pressed, stop the arms
		if(noConfs) {
			double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT],
                        SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
			return;
		}
	}

	// Check the b for each arm and apply velocities accordingly
	// For left: 4 or 6, for right: 5 or 7, lower arm button is smaller (4 or 5)
	somatic_motor_t* arm [] = {krang->arms[Krang::LEFT], krang->arms[Krang::RIGHT]};
	for(size_t arm_idx = 0; arm_idx < 2; arm_idx++) {

		// Initialize the input
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Change the input based on the lower or higher button input
		bool inputSet = true;
		size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
		if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
		else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));
		else inputSet = false;

		// Set the input for this arm
		if(inputSet) somatic_motor_cmd(&daemon_cx, arm[arm_idx],
                                    SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
	}
}
