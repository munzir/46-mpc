# 35-balancing

This repo contains code for balancing krang. Aim is to implement original balancing logic (repo 28) such that it is more readable and maintainable.
 
## Dependencies

- [DART](https://dartsim.github.io/install_dart_on_ubuntu.html) - Use 'apt install' instructions on the page.
- [09-URDF](https://github.gatech.edu/WholeBodyControlAttempt1/09-URDF) - Clone this repo. No installation needed.
- [18-OnlineCoM](https://github.gatech.edu/WholeBodyControlAttempt1/18-OnlineCoM) - Clone this repo. No installation needed.
- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic) - Follow installation instructions on the git readme.
- [36-kore (newdart branch)](https://github.gatech.edu/WholeBodyControlAttempt1/36-kore/tree/newdart) - Follow installation instructions on the git readme

## Installation

In params.cfg, give absolute paths to local locations of the file for 'urdfpath' and 'comParametersPath'. They are found in repo 09-URDF and 18-OnlineCoM respectively.

Then, follow the steps to install

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
