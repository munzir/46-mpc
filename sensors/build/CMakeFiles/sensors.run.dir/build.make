# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors/build

# Utility rule file for sensors.run.

# Include the progress variables for this target.
include CMakeFiles/sensors.run.dir/progress.make

CMakeFiles/sensors.run:
	./sensors

sensors.run: CMakeFiles/sensors.run
sensors.run: CMakeFiles/sensors.run.dir/build.make
.PHONY : sensors.run

# Rule to build all files generated by this target.
CMakeFiles/sensors.run.dir/build: sensors.run
.PHONY : CMakeFiles/sensors.run.dir/build

CMakeFiles/sensors.run.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensors.run.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensors.run.dir/clean

CMakeFiles/sensors.run.dir/depend:
	cd /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors/build /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors/build /home/cerdogan/Documents/Krang/Software/project/krang/demos/sensors/build/CMakeFiles/sensors.run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensors.run.dir/depend

