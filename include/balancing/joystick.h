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
 * @file joystick.h
 * @author Munzir Zafar
 * @date Oct 30, 2018
 * @brief Header for joystick.cpp that reads data from the joystick
 * channel
 */

#ifndef KRANG_BALANCING_JOYSTICK_H_
#define KRANG_BALANCING_JOYSTICK_H_

#include <ach.h>
#include <somatic.h>

class Joystick {
 public:
  enum FingerButtons {
    L1,
    L2,
    R1,
    R2,
    L1L2,
    R1R2,
    L1R1,
    L1R2,
    L2R1,
    L2R2,
    L1L2R1,
    L1L2R2,
    L1R1R2,
    L2R1R2,
    L1L2R1R2,
    L1L2R1R2_FREE
  };

  enum RightThumb {
    B1_PRESS,
    B1_HOLD,
    B1_RELEASE,
    B2_PRESS,
    B2_HOLD,
    B2_RELEASE,
    B3_PRESS,
    B3_HOLD,
    B3_RELEASE,
    B4_PRESS,
    B4_HOLD,
    B4_RELEASE,
    B10_PRESS,
    B10_HOLD,
    B10_RELEASE,
    RIGHT_THUMB_HORZ_PRESS,
    RIGHT_THUMB_HORZ_HOLD,
    RIGHT_THUMB_HORZ_RELEASE,
    RIGHT_THUMB_VERT_PRESS,
    RIGHT_THUMB_VERT_HOLD,
    RIGHT_THUMB_VERT_RELEASE,
    RIGHT_THUMB_FREE
  };

  enum LeftThumb {
    B9_PRESS,
    B9_HOLD,
    B9_RELEASE,
    LEFT_THUMB_HORZ_PRESS,
    LEFT_THUMB_HORZ_HOLD,
    LEFT_THUMB_HORZ_RELEASE,
    LEFT_THUMB_VERT_PRESS,
    LEFT_THUMB_VERT_HOLD,
    LEFT_THUMB_VERT_RELEASE,
    CURSOR_HORZ_PRESS,
    CURSOR_HORZ_HOLD,
    CURSOR_HORZ_RELEASE,
    CURSOR_VERT_PRESS,
    CURSOR_VERT_HOLD,
    CURSOR_VERT_RELEASE,
    LEFT_THUMB_FREE
  };

  enum { LEFT, RIGHT };
  Joystick();
  ~Joystick(){};

  FingerButtons fingerMode;
  RightThumb rightMode;
  LeftThumb leftMode;
  double thumbValue[2];

  /* ************************************************************************ */
  /// Update joystick state
  bool Update();

 private:
  /* ************************************************************************ */
  // Opens ach channel to read joystick data
  void OpenJoystickChannel();

  /* ************************************************************************ */
  // Maps the data read from ach channels to JoystickState
  void MapToJoystickState(char* b, double* x);

  ach_channel_t ach_chan;  ///< Read joystick data on this channel
};
#endif  // KRANG_BALANCING_JOYSTICK_H_
