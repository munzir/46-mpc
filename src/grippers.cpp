/* ********************************************************************************************* */
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
 * @file grippers.cpp
 * @author Munzir Zafar
 * @date Oct 30, 2018
 * @brief Implements control of grippers based on joystick Input
 */

#include "grippers.h"

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic/motor.h>

#include <kore.hpp>
#include <kore/util.hpp>

/* ************************************************************************************/
/// Handles the joystick commands for the left/right Schunk grippers
void controlSchunkGrippers (somatic_d_t& daemon_cx, const char* b, const double* x,
                            Krang::Hardware* krang) {

	// Button 4 with top/down at the right circular thingy indicates a motion for the left gripper
	double dq [] = {0.0};
	dq[0] = x[3] * 10.0;
	if(b[4])
		somatic_motor_cmd(&daemon_cx, krang->grippers[Krang::LEFT], SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 1, NULL);

	// Button 5 with the same circular thingy for the right gripper
	if(b[5])
		somatic_motor_cmd(&daemon_cx, krang->grippers[Krang::RIGHT], SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 1, NULL);
}


