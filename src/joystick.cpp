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
 * @file joystick.cpp
 * @author Munzir Zafar
 * @date Oct 30, 2018
 * @brief Reading data from the joystick channel
 */

#include "joystick.h"

#include <cmath>
#include <somatic.h>
#include <ach.h>

/* ******************************************************************************************** */
// Order in which buttons of joystick appear on the data arrays from ach channel
enum BUTTONS {
  ONE = 0,
  TWO,
  THREE,
  FOUR,
  LEFT1,
  RIGHT1,
  LEFT2,
  RIGHT2,
  NINE,
  TEN
} ;

enum ANALOG {
  LEFT_THUMB_HORZ = 0,
  LEFT_THUMB_VERT,
  RIGHT_THUMB_HORZ,
  RIGHT_THUMB_VERT,
  CURSOR_HORZ,
  CURSOR_VERT
} ;

Joystick::Joystick () {
  OpenJoystickChannel();
}
/* ***************************************************************************** */
// Opens ach channel to read joystick data
void Joystick::OpenJoystickChannel() {
  // Initialize the joystick channel
  int r = ach_open(&ach_chan, "joystick-data", NULL);
  aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n",
    ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
}


/* ***************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick
bool Joystick::Update() {

  // Get the message and check output is OK.
  int r = 0;
  Somatic__Joystick *js_msg =
      SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, NULL, 4096, &ach_chan );
  if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

  // Get the values
  char b[10];
  for(size_t i = 0; i < 10; i++)
    b[i] = js_msg->buttons->data[i] ? 1 : 0;

  double x[6];
  for(size_t i = 0; i < 6; i++)
    x[i] = js_msg->axes->data[i];

  // Free the joystick message
  somatic__joystick__free_unpacked(js_msg, NULL);

  MapToJoystickState(b, x);
  return true;
}

/* ***************************************************************************** */
void Joystick::MapToJoystickState(char* b, double* x) {
  //b={(1),(2),(3),(4),L1,L2,R1,R2,(9),(10)}
  //b={  0,  0,  0,  0, 0, 0, 0, 0,  0,  1 }
  static char last_b[10] = {0};
  static bool last_bool_x[6] = {0};
  bool bool_x[6];

  // convert analog values to boolean data - 0/1
  for (int i=0; i < 6; i++) {
    bool_x[i] = (fabs(x[i]) > 0.001);
  }

  // thumbValues to be zero if thumb axes are not active
  thumbValue[Joystick::LEFT] = 0.0;
  thumbValue[Joystick::RIGHT] = 0.0;

  // for one horz/vert pair, only one direction can be active at a time
  // prioritize the one with a higher value
  if(bool_x[LEFT_THUMB_HORZ] == 1 && bool_x[LEFT_THUMB_VERT] == 1) {
    if(fabs(x[LEFT_THUMB_HORZ]) >= fabs(x[LEFT_THUMB_VERT])) {
      bool_x[LEFT_THUMB_HORZ] = 1;
      bool_x[LEFT_THUMB_VERT] = 0;
    } else {
      bool_x[LEFT_THUMB_HORZ] = 0;
      bool_x[LEFT_THUMB_VERT] = 1;
    }
  }
  if(bool_x[RIGHT_THUMB_HORZ] == 1 && bool_x[RIGHT_THUMB_VERT] == 1) {
    if(fabs(x[RIGHT_THUMB_HORZ]) >= fabs(x[RIGHT_THUMB_VERT])) {
      bool_x[RIGHT_THUMB_HORZ] = 1;
      bool_x[RIGHT_THUMB_VERT] = 0;
    } else {
      bool_x[RIGHT_THUMB_HORZ] = 0;
      bool_x[RIGHT_THUMB_VERT] = 1;
    }
  }
  if(bool_x[CURSOR_HORZ] == 1 && bool_x[CURSOR_VERT] == 1) {
    if(fabs(x[CURSOR_HORZ]) >= fabs(x[CURSOR_VERT])) {
      bool_x[CURSOR_HORZ] = 1;
      bool_x[CURSOR_VERT] = 0;
    } else {
      bool_x[CURSOR_HORZ] = 0;
      bool_x[CURSOR_VERT] = 1;
    }
  }

  // RIGHMB MODEITAL BUTTONS [(1), (2), (3), (4), (10)]
  // Button (1)
  if (b[ONE] == 1 && last_b[ONE] == 0) {  //tapping (1)
    rightMode = Joystick::B1_PRESS;
  }
  else if (b[ONE] == 1  && last_b[ONE] == 1) {  //holding (1)
    rightMode = Joystick::B1_HOLD;
  }
  else if (b[ONE] == 0 && last_b[ONE] == 1) { // releasing (1)
    rightMode = Joystick::B1_RELEASE;
  }
  // Button (2)
  else if (b[TWO] == 1 && last_b[TWO] == 0) {  //tapping (2)
    rightMode = Joystick::B2_PRESS;
  }
  else if (b[TWO] == 1  && last_b[TWO] == 1) {  //holding (2)
    rightMode = Joystick::B2_HOLD;
  }
  else if (b[TWO] == 0 && last_b[TWO] == 1) { // releasing (2)
    rightMode = Joystick::B2_RELEASE;
  }
  // Button (3)
  else if (b[THREE] == 1 && last_b[THREE] == 0) {  //tapping (3)
    rightMode = Joystick::B3_PRESS;
  }
  else if (b[THREE] == 1  && last_b[THREE] == 1) {  //holding (3)
    rightMode = Joystick::B3_HOLD;
  }
  else if (b[THREE] == 0 && last_b[THREE] == 1) { // releasing (3)
    rightMode = Joystick::B3_RELEASE;
  }
  // Button (4)
  else if (b[FOUR] == 1 && last_b[FOUR] == 0) {  //tapping (4)
    rightMode = Joystick::B4_PRESS;
  }
  else if (b[FOUR] == 1  && last_b[FOUR] == 1) {  //holding (4)
    rightMode = Joystick::B4_HOLD;
  }
  else if (b[FOUR] == 0 && last_b[FOUR] == 1) { // releasing (4)
    rightMode = Joystick::B4_RELEASE;
  }
  // Button (10)
  else if (b[TEN] == 1 && last_b[TEN] == 0) {  //tapping (10)
    rightMode = Joystick::B10_PRESS;
  }
  else if (b[TEN] == 1  && last_b[TEN] == 1) {  //holding (10)
    rightMode = Joystick::B10_HOLD;
  }
  else if (b[TEN] == 0 && last_b[TEN] == 1) { // releasing (10)
    rightMode = Joystick::B10_RELEASE;
  }
  // RIGHT THUMB MODE: ANALOG
  //tapping TSR-side
  else if ( bool_x[RIGHT_THUMB_HORZ] == 1 &&  last_bool_x[RIGHT_THUMB_HORZ] == 0  ) {
    rightMode = Joystick::RIGHT_THUMB_HORZ_PRESS;
    thumbValue[Joystick::RIGHT] = x[RIGHT_THUMB_HORZ];
  }
  //holding TSR-side
  else if ( bool_x[RIGHT_THUMB_HORZ] == 1 &&  last_bool_x[RIGHT_THUMB_HORZ] == 1  ) {
    rightMode = Joystick::RIGHT_THUMB_HORZ_HOLD;
    thumbValue[Joystick::RIGHT] = x[RIGHT_THUMB_HORZ];
  }
  //releasing TSR-side
  else if ( bool_x[RIGHT_THUMB_HORZ] == 0 &&  last_bool_x[RIGHT_THUMB_HORZ] == 1  ) {
    rightMode = Joystick::RIGHT_THUMB_HORZ_RELEASE;
  }
  //tapping TSR-updown
  else if ( bool_x[RIGHT_THUMB_VERT] == 1 &&  last_bool_x[RIGHT_THUMB_VERT] == 0  ) {
    rightMode = Joystick::RIGHT_THUMB_VERT_PRESS;
    thumbValue[Joystick::RIGHT] = x[RIGHT_THUMB_VERT];
  }
  //holding TSR-updown
  else if ( bool_x[RIGHT_THUMB_VERT] == 1 &&  last_bool_x[RIGHT_THUMB_VERT] == 1  ) {
    rightMode = Joystick::RIGHT_THUMB_VERT_HOLD;
    thumbValue[Joystick::RIGHT] = x[RIGHT_THUMB_VERT];
  }
  //releasing TSR-updown
  else if ( bool_x[RIGHT_THUMB_VERT] == 0 &&  last_bool_x[RIGHT_THUMB_VERT] == 1  ) {
    rightMode = Joystick::RIGHT_THUMB_VERT_RELEASE;
  }
  // no button pressed
  else if ( b[ONE] == 0 && b[TWO] == 0 &&
            b[THREE] == 0 && b[FOUR] == 0 &&
            b[TEN] == 0 &&
            bool_x[RIGHT_THUMB_HORZ] == 0 &&
            bool_x[RIGHT_THUMB_VERT] == 0) {
      rightMode = Joystick::RIGHT_THUMB_FREE;
  }




  // LEFT THUMB MODE: DIGITAL BUTTONS [(9)]
  // Button (9)
  if (b[NINE] == 1 && last_b[NINE] == 0) {  //tapping (9)
    leftMode = Joystick::B9_PRESS;
  }
  else if (b[NINE] == 1  && last_b[NINE] == 1) {  //holding (9)
    leftMode = Joystick::B9_HOLD;
  }
  else if (b[NINE] == 0 && last_b[NINE] == 1) { // releasing (9)
    leftMode = Joystick::B9_RELEASE;
  }
  // LEFT THUMB MODE: ANALOG
  //tapping TSL-side
  else if ( bool_x[LEFT_THUMB_HORZ] == 1 &&  last_bool_x[LEFT_THUMB_HORZ] == 0  ) {
    leftMode = Joystick::LEFT_THUMB_HORZ_PRESS;
    thumbValue[Joystick::LEFT] = x[LEFT_THUMB_HORZ];
  }
  //holding TSL-side
  else if ( bool_x[LEFT_THUMB_HORZ] == 1 &&  last_bool_x[LEFT_THUMB_HORZ] == 1  ) {
    leftMode = Joystick::LEFT_THUMB_HORZ_HOLD;
    thumbValue[Joystick::LEFT] = x[LEFT_THUMB_HORZ];
  }
  //releasing TSL-side
  else if ( bool_x[LEFT_THUMB_HORZ] == 0 &&  last_bool_x[LEFT_THUMB_HORZ] == 1  ) {
    leftMode = Joystick::LEFT_THUMB_HORZ_RELEASE;
  }
  //tapping TSL-updn
  else if ( bool_x[LEFT_THUMB_VERT] == 1 &&  last_bool_x[LEFT_THUMB_VERT] == 0  ) {
    leftMode = Joystick::LEFT_THUMB_VERT_PRESS;
    thumbValue[Joystick::LEFT] = x[LEFT_THUMB_VERT];
  }
  //holding TSL-updn
  else if ( bool_x[LEFT_THUMB_VERT] == 1 &&  last_bool_x[LEFT_THUMB_VERT] == 1  ) {
    leftMode = Joystick::LEFT_THUMB_VERT_HOLD;
    thumbValue[Joystick::LEFT] = x[LEFT_THUMB_VERT];
  }
  //releasing TSL-updn
  else if ( bool_x[LEFT_THUMB_VERT] == 0 &&  last_bool_x[LEFT_THUMB_VERT] == 1  ) {
    leftMode = Joystick::LEFT_THUMB_VERT_RELEASE;
  }
  //tapping AXL-side
  else if ( bool_x[CURSOR_HORZ] == 1 &&  last_bool_x[CURSOR_HORZ] == 0  ) {
    leftMode = Joystick::CURSOR_HORZ_PRESS;
    thumbValue[Joystick::LEFT] = x[CURSOR_HORZ];
  }
  //holding AXL-side
  else if ( bool_x[CURSOR_HORZ] == 1 &&  last_bool_x[CURSOR_HORZ] == 1  ) {
    leftMode = Joystick::CURSOR_HORZ_HOLD;
    thumbValue[Joystick::LEFT] = x[CURSOR_HORZ];
  }
  //releasing AXL-side
  else if ( bool_x[CURSOR_HORZ] == 0 &&  last_bool_x[CURSOR_HORZ] == 1  ) {
    leftMode = Joystick::CURSOR_HORZ_RELEASE;
  }
  //tapping AXL-updn
  else if ( bool_x[CURSOR_VERT] == 1 &&  last_bool_x[CURSOR_VERT] == 0  ) {
    leftMode = Joystick::CURSOR_VERT_PRESS;
    thumbValue[Joystick::LEFT] = x[CURSOR_VERT];
  }
  //holding AXL-updn
  else if ( bool_x[CURSOR_VERT] == 1 &&  last_bool_x[CURSOR_VERT] == 1  ) {
    leftMode = Joystick::CURSOR_VERT_HOLD;
    thumbValue[Joystick::LEFT] = x[CURSOR_VERT];
  }
  //releasing AXL-updn
  else if ( bool_x[CURSOR_VERT] == 0 &&  last_bool_x[CURSOR_VERT] == 1  ) {
    leftMode = Joystick::CURSOR_VERT_RELEASE;
  }
  // no button pressed
  else if ( b[NINE] == 0 &&
            bool_x[LEFT_THUMB_HORZ] == 0 &&
            bool_x[LEFT_THUMB_VERT] == 0 &&
            bool_x[CURSOR_HORZ] == 0 &&
            bool_x[CURSOR_VERT] == 0 ) {
      leftMode = Joystick::LEFT_THUMB_FREE;
  }

  // FINGER MODE: BUTTONS [ L1, L2, R1, R2]
  // pressing L1
  if      (b[LEFT1] == 1 && b[LEFT2] == 0  && b[RIGHT1] == 0  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::L1;
  }
  // pressing L2
  else if (b[LEFT1] == 0 && b[LEFT2] == 1  && b[RIGHT1] == 0  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::L2;
  }
  // pressing R1
  else if (b[LEFT1] == 0 && b[LEFT2] == 0  && b[RIGHT1] == 1  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::R1;
  }
  // pressing R2
  else if (b[LEFT1] == 0 && b[LEFT2] == 0  && b[RIGHT1] == 0  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::R2;
  }
  // pressing L1 & L2
  else if (b[LEFT1] == 1 && b[LEFT2] == 1  && b[RIGHT1] == 0  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::L1L2;
  }
  // pressing R1 & R2
  else if (b[LEFT1] == 0 && b[LEFT2] == 0  && b[RIGHT1] == 1  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::R1R2;
  }
  // pressing L1 & R1
  else if (b[LEFT1] == 1 && b[LEFT2] == 0  && b[RIGHT1] == 1  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::L1R1;
  }
  // pressing L1 & R2
  else if (b[LEFT1] == 1 && b[LEFT2] == 0  && b[RIGHT1] == 0  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::L1R2;
  }
  // pressing L2 & R1
  else if (b[LEFT1] == 0 && b[LEFT2] == 1  && b[RIGHT1] == 1  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::L2R1;
  }
  // pressing L2 & R2
  else if (b[LEFT1] == 0 && b[LEFT2] == 1  && b[RIGHT1] == 0  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::L2R2;
  }
  // pressing L1, L2, & R1
  else if (b[LEFT1] == 1 && b[LEFT2] == 1  && b[RIGHT1] == 1  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::L1L2R1;
  }
  // pressing L1, L2, & R2
  else if (b[LEFT1] == 1 && b[LEFT2] == 1  && b[RIGHT1] == 0  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::L1L2R2;
  }
  // pressing L1, R1, & R2
  else if (b[LEFT1] == 1 && b[LEFT2] == 0  && b[RIGHT1] == 1  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::L1R1R2;
  }
  // pressing L2, R1, & R2
  else if (b[LEFT1] == 0 && b[LEFT2] == 1  && b[RIGHT1] == 1  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::L2R1R2;
  }
  // pressing L1, L2, R1, & R2
  else if (b[LEFT1] == 1 && b[LEFT2] == 1  && b[RIGHT1] == 1  && b[RIGHT2] == 1 ) {
    fingerMode = Joystick::L1L2R1R2;
  }
  // pressing No Buttons
  else if (b[LEFT1] == 0 && b[LEFT2] == 0  && b[RIGHT1] == 0  && b[RIGHT2] == 0 ) {
    fingerMode = Joystick::L1L2R1R2_FREE;
  }


  // update last values for buttons and analog values
  for (int i=0; i < 10; i++) {
    last_b[i] = b[i];
  }
  for (int i=0; i < 6; i++) {
    last_bool_x[i] = bool_x[i];
  }
}
