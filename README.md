# 35-balancing

This repo contains code for balancing krang. Aim is to implement original balancing logic (repo 28) such that it is more readable and maintainable.
 
## Dependencies

- [DART](https://dartsim.github.io/install_dart_on_ubuntu.html) - Use 'apt install' instructions on the page.
- [09-URDF](https://github.gatech.edu/WholeBodyControlAttempt1/09-URDF) - Clone this repo. No installation needed.
- [18-OnlineCoM](https://github.gatech.edu/WholeBodyControlAttempt1/18-OnlineCoM) - Clone this repo. No installation needed.
- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic) - Follow installation instructions on the git readme.
- [36-kore (newdart branch)](https://github.gatech.edu/WholeBodyControlAttempt1/36-kore/tree/newdart) - Follow installation instructions on the git readme

## Compilation

In params.cfg, give absolute paths to local locations of the file for 'urdfpath' and 'comParametersPath'. They are found in repo 09-URDF and 18-OnlineCoM respectively.

Then, follow the steps to install

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
    

## Usage

In order to run with a simulation, follow instructions in [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach) to launch the ach channels and processes required before this program is run. Then in the build folder of this repo, type:
 
    sudo ./01-balancing

Press 'Enter' for the program to start running. Press 's' then 'Enter' to enable wheel control. Use joystick and keyboard to manipulate the robot. I will write instructions on joystick and keyboard functions later. For now, refer to 'events.cpp' file to see what buttons of joystick and keyboard perform what functionality. 
