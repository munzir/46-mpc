# 35-balancing

This repo contains code for balancing krang. Aim is to implement original balancing logic (repo 28) such that it is more readable and maintainable.

## Dependencies

- DART
 [Dart Homepage](https://dartsim.github.io)

- config4cpp
 [Source Code Download](http://www.config4star.org/#main-source-code)

 How to add config4cpp to your system (Linux/BSD/OSX)

  1: Download and extract the source code of config4cpp

  2: cd config4cpp

  3: Follow the README.txt in config4cpp/

  4: Run the following commands to add the make'd files into your local system

    sudo cp bin/{config2cpp,config4cpp} /usr/local/bin &&
    sudo cp lib/libconfig4cpp.a /usr/local/lib &&
    sudo cp -r include/config4cpp /usr/local/include &&
    sudo chmod g+rx /usr/local/include/config4cpp &&
    sudo chmod o+rx /usr/local/include/config4cpp

  \*Note: You can just copy paste the above block of commands

- [09-URDF](https://github.gatech.edu/WholeBodyControlAttempt1/09-URDF)
 Install the repository.

- [18-OnlineCoM](https://github.gatech.edu/WholeBodyControlAttempt1/18-OnlineCoM)  Install the repository.

- [36-kore](https://github.gatech.edu/WholeBodyControlAttempt1/36-kore/tree/newdart)
 Install the repository.

- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic)
 Install the repository.

- [44a-krach](https://github.gatech.edu/WholeBodyControlAttempt1/44a-krach)
 Install the repository.

### Optional Dependency

For simulation:

- [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach)
 Install the repository.

## Compilation

    mkdir build
    cd build
    cmake ..
    make

## Usage

In order to run with a simulation, follow instructions in [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach) to launch the ach channels and processes required before this program is run. Then in the build folder of this repo, type:

    sudo ./01-balancing

Press 'Enter' for the program to start running. Press 's' then 'Enter' to enable wheel control. Use joystick and keyboard to manipulate the robot. I will write instructions on joystick and keyboard functions later. For now, refer to 'events.cpp' file to see what buttons of joystick and keyboard perform what functionality.
