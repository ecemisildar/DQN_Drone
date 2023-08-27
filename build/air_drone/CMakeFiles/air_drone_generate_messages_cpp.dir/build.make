# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ecem/drone_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ecem/drone_ws/build

# Utility rule file for air_drone_generate_messages_cpp.

# Include the progress variables for this target.
include air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/progress.make

air_drone/CMakeFiles/air_drone_generate_messages_cpp: /home/ecem/drone_ws/devel/include/air_drone/Pose.h
air_drone/CMakeFiles/air_drone_generate_messages_cpp: /home/ecem/drone_ws/devel/include/air_drone/MotorSpeed.h


/home/ecem/drone_ws/devel/include/air_drone/Pose.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ecem/drone_ws/devel/include/air_drone/Pose.h: /home/ecem/drone_ws/src/air_drone/msg/Pose.msg
/home/ecem/drone_ws/devel/include/air_drone/Pose.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ecem/drone_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from air_drone/Pose.msg"
	cd /home/ecem/drone_ws/src/air_drone && /home/ecem/drone_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ecem/drone_ws/src/air_drone/msg/Pose.msg -Iair_drone:/home/ecem/drone_ws/src/air_drone/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p air_drone -o /home/ecem/drone_ws/devel/include/air_drone -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ecem/drone_ws/devel/include/air_drone/MotorSpeed.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ecem/drone_ws/devel/include/air_drone/MotorSpeed.h: /home/ecem/drone_ws/src/air_drone/msg/MotorSpeed.msg
/home/ecem/drone_ws/devel/include/air_drone/MotorSpeed.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ecem/drone_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from air_drone/MotorSpeed.msg"
	cd /home/ecem/drone_ws/src/air_drone && /home/ecem/drone_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ecem/drone_ws/src/air_drone/msg/MotorSpeed.msg -Iair_drone:/home/ecem/drone_ws/src/air_drone/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p air_drone -o /home/ecem/drone_ws/devel/include/air_drone -e /opt/ros/noetic/share/gencpp/cmake/..

air_drone_generate_messages_cpp: air_drone/CMakeFiles/air_drone_generate_messages_cpp
air_drone_generate_messages_cpp: /home/ecem/drone_ws/devel/include/air_drone/Pose.h
air_drone_generate_messages_cpp: /home/ecem/drone_ws/devel/include/air_drone/MotorSpeed.h
air_drone_generate_messages_cpp: air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/build.make

.PHONY : air_drone_generate_messages_cpp

# Rule to build all files generated by this target.
air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/build: air_drone_generate_messages_cpp

.PHONY : air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/build

air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/clean:
	cd /home/ecem/drone_ws/build/air_drone && $(CMAKE_COMMAND) -P CMakeFiles/air_drone_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/clean

air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/depend:
	cd /home/ecem/drone_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ecem/drone_ws/src /home/ecem/drone_ws/src/air_drone /home/ecem/drone_ws/build /home/ecem/drone_ws/build/air_drone /home/ecem/drone_ws/build/air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : air_drone/CMakeFiles/air_drone_generate_messages_cpp.dir/depend

