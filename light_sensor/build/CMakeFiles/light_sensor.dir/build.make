# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/paolo/custom_sensors/light_sensor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/paolo/custom_sensors/light_sensor/build

# Include any dependencies generated for this target.
include CMakeFiles/light_sensor.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/light_sensor.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/light_sensor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/light_sensor.dir/flags.make

CMakeFiles/light_sensor.dir/LightSensor.cc.o: CMakeFiles/light_sensor.dir/flags.make
CMakeFiles/light_sensor.dir/LightSensor.cc.o: ../LightSensor.cc
CMakeFiles/light_sensor.dir/LightSensor.cc.o: CMakeFiles/light_sensor.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/paolo/custom_sensors/light_sensor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/light_sensor.dir/LightSensor.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/light_sensor.dir/LightSensor.cc.o -MF CMakeFiles/light_sensor.dir/LightSensor.cc.o.d -o CMakeFiles/light_sensor.dir/LightSensor.cc.o -c /home/paolo/custom_sensors/light_sensor/LightSensor.cc

CMakeFiles/light_sensor.dir/LightSensor.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/light_sensor.dir/LightSensor.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/paolo/custom_sensors/light_sensor/LightSensor.cc > CMakeFiles/light_sensor.dir/LightSensor.cc.i

CMakeFiles/light_sensor.dir/LightSensor.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/light_sensor.dir/LightSensor.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/paolo/custom_sensors/light_sensor/LightSensor.cc -o CMakeFiles/light_sensor.dir/LightSensor.cc.s

# Object files for target light_sensor
light_sensor_OBJECTS = \
"CMakeFiles/light_sensor.dir/LightSensor.cc.o"

# External object files for target light_sensor
light_sensor_EXTERNAL_OBJECTS =

liblight_sensor.so: CMakeFiles/light_sensor.dir/LightSensor.cc.o
liblight_sensor.so: CMakeFiles/light_sensor.dir/build.make
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libignition-sensors6.so.6.8.0
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libignition-common4.so.4.7.0
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libignition-transport11.so.11.4.1
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libuuid.so
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libuuid.so
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libsdformat12.so.12.7.2
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libignition-utils1.so.1.5.1
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libignition-msgs8.so.8.7.0
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.15.1
liblight_sensor.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
liblight_sensor.so: CMakeFiles/light_sensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/paolo/custom_sensors/light_sensor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library liblight_sensor.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/light_sensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/light_sensor.dir/build: liblight_sensor.so
.PHONY : CMakeFiles/light_sensor.dir/build

CMakeFiles/light_sensor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/light_sensor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/light_sensor.dir/clean

CMakeFiles/light_sensor.dir/depend:
	cd /home/paolo/custom_sensors/light_sensor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/paolo/custom_sensors/light_sensor /home/paolo/custom_sensors/light_sensor /home/paolo/custom_sensors/light_sensor/build /home/paolo/custom_sensors/light_sensor/build /home/paolo/custom_sensors/light_sensor/build/CMakeFiles/light_sensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/light_sensor.dir/depend

