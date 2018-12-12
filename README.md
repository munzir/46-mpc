# 35-balancing

This repo contains code for balancing krang. Aim is to implement original balancing logic (repo 28) such that it is more readable and maintainable.
 
## Dependencies

- [DART](https://dartsim.github.io/install_dart_on_ubuntu.html) - Use 'apt install' instructions on the page. Along with the basic library also install dart6-utils-dev, dart6-gui-dev and dart6-utils-urdf-dev
- [config4cpp](http://www.config4star.org/) - Using the link, download the source code for Config4Cpp (C++ version) using the 'compressed tar' option. After extracting, cd into the directory and

      make
  This creates files in bin, lib and include folders that we will manually copy to the system folder. Before doing so, make sure permissions of the files are set to "-rwxr-xr-x" for all files, and to "drwxr-xr-x" for the directory. You can check this using:
      
      ls -la 
  inside the respective folders. This will list the files along with their permissions. If the permission is not the same as mentioned above, change the permission using chmod 755 <file-name>. For example, if doing "ls -la" in the include folder gives the output:
      
      drwx------ 2 munzir munzir 4096 Feb  2  2012 config4cpp
  then we need to change the permission for this folder using
  
      chmod 755 config4cpp
  Once the permissions are properly set, then copy files from bin folder to /usr/local/bin/, from lib folder to /usr/local/lib/, and the config4cpp directory in the include folder to /usr/local/include. Once the files are copied, do:
  
      ldconfig
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
