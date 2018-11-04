#include "joystick.h"
#include <iomanip>
#include <somatic.h>
#include <somatic/daemon.h>

int main () {

  // Initialize the daemon
  //somatic_d_t daemon_cx;				///< The context of the current daemon
  //somatic_d_opts_t dopt;
  //memset(&dopt, 0, sizeof(dopt));
  //dopt.ident = "02-joystick_testing";
  //somatic_d_init(&daemon_cx, &dopt);
  openJoystickChannel();

  JoystickState joystick_state, last_joystick_state;
  char b[10];
  double x[6];
  const char finger_button_strings[][24] = {
        "L1",
        "L2",
        "R1",
        "R2",
        "L1L2",
        "R1R2",
        "L1R1",
        "L1R2",
        "L2R1",
        "L2R2",
        "L1L2R1",
        "L1L2R2",
        "L1R1R2",
        "L2R1R2",
        "L1L2R1R2",
        "L1L2R1R2_FREE"
  } ;

  const char right_thumb_strings[][36] = {
        "B1_PRESS", "B1_HOLD", "B1_RELEASE",
        "B2_PRESS", "B2_HOLD", "B2_RELEASE",
        "B3_PRESS", "B3_HOLD", "B3_RELEASE",
        "B4_PRESS", "B4_HOLD", "B4_RELEASE",
        "B10_PRESS", "B10_HOLD", "B10_RELEASE",
        "RIGHT_THUMB_HORZ_PRESS", "RIGHT_THUMB_HORZ_HOLD", "RIGHT_THUMB_HORZ_RELEASE",
        "RIGHT_THUMB_VERT_PRESS", "RIGHT_THUMB_VERT_HOLD", "RIGHT_THUMB_VERT_RELEASE",
        "RIGHT_THUMB_FREE"
  } ;

  const char left_thumb_strings[][36] = {
        "B9_PRESS", "B9_HOLD", "B9_RELEASE",
        "LEFT_THUMB_HORZ_PRESS", "LEFT_THUMB_HORZ_HOLD", "LEFT_THUMB_HORZ_RELEASE",
        "LEFT_THUMB_VERT_PRESS", "LEFT_THUMB_VERT_HOLD", "LEFT_THUMB_VERT_RELEASE",
        "CURSOR_HORZ_PRESS", "CURSOR_HORZ_HOLD", "CURSOR_HORZ_RELEASE",
        "CURSOR_VERT_PRESS", "CURSOR_VERT_HOLD", "CURSOR_VERT_RELEASE",
        "LEFT_THUMB_FREE"
  };

  int c = 0;
  while(!somatic_sig_received) {
    getJoystickInput(b, x, &joystick_state);
    if(joystick_state.fingerMode != last_joystick_state.fingerMode ||
       joystick_state.rightMode != last_joystick_state.rightMode ||
       joystick_state.leftMode != last_joystick_state.leftMode) {
      std::cout << std::endl << std::endl;
      std::cout << "Finger     : " << finger_button_strings[joystick_state.fingerMode] << std::endl;
      std::cout << "Right Thumb: " << right_thumb_strings[joystick_state.rightMode] << std::endl;
      std::cout << "Left Thumb : " << left_thumb_strings[joystick_state.leftMode] << std::endl;
    }
    std::cout << "\rLeft Thumb Value: ";
    std::cout << std::setprecision(4) << joystick_state.thumbValue[JoystickState::LEFT];
    std::cout << "  Right Thumb Value: ";
    std::cout << std::setprecision(4) << joystick_state.thumbValue[JoystickState::RIGHT];
    std::cout << "                    ";
    last_joystick_state.fingerMode = joystick_state.fingerMode;
    last_joystick_state.rightMode = joystick_state.rightMode;
    last_joystick_state.leftMode = joystick_state.leftMode;
  }

}
