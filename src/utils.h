//
// Created by Sherlock Hummus on 8/12/18.
//

#ifndef DEB8_DEMOS_UTILS_H
#define DEB8_DEMOS_UTILS_H

using namespace Eigen;
/* ******************************************************************************************** */
// For logging purposes
int Krang::COLOR_RED_BACKGROUND = 11;
int Krang::COLOR_YELLOW_BACKGROUND = 12;
int Krang::COLOR_GREEN_BACKGROUND = 13;
int Krang::COLOR_WHITE_BACKGROUND = 14;
int Krang::curses_display_row = 30;
int Krang::curses_display_precision = 15;
bool Krang::doing_curses = false;

struct LogState {

    // Read state and sensors
    double time;
    Vector3d com;
    double averagedTorque, torque;
    double amcLeft, amcRight;

    // Controller stuff
    Vector6d state, refState;
    double lastUleft, lastUright;

    /// Constructor
    LogState (double t, const Vector3d& c, double aTo, double to, double aL, double aR,
              const Vector6d& s, const Vector6d& rS, double lUl, double lUr) :
            time(t), com(c), averagedTorque(aTo), torque(to), amcLeft(aL), amcRight(aR), state(s),
            refState(rS), lastUleft(lUl), lastUright(lUr) {}

    /// Print
    void print () {
        //       torques         currents        state        refstate       time
        printf("%lf\t%lf\t  %lf\t%lf\t%lf\t%lf\t  %lf\t%lf\t%lf\t %lf\t%lf\t%lf\t %lf\n",
               averagedTorque, torque, lastUleft, lastUright, amcLeft, amcRight,
               state(0)*180.0/M_PI, state(2)*180.0/M_PI, state(4)*180.0/M_PI, refState(0)*180.0/M_PI,
               refState(2)*180.0/M_PI, refState(4)*180.0/M_PI, time);
    }

};
#endif //DEB8_DEMOS_UTILS_H
