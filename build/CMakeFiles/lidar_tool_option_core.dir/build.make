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
CMAKE_SOURCE_DIR = /home/ahmet/Lidar_Simulation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ahmet/Lidar_Simulation/build

# Include any dependencies generated for this target.
include CMakeFiles/lidar_tool_option_core.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lidar_tool_option_core.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lidar_tool_option_core.dir/flags.make

CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.o: CMakeFiles/lidar_tool_option_core.dir/flags.make
CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.o: ../src/Lidar_Tool_Option/Lidar_Tool_Option.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ahmet/Lidar_Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.o -c /home/ahmet/Lidar_Simulation/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp

CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ahmet/Lidar_Simulation/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp > CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.i

CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ahmet/Lidar_Simulation/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp -o CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.s

# Object files for target lidar_tool_option_core
lidar_tool_option_core_OBJECTS = \
"CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.o"

# External object files for target lidar_tool_option_core
lidar_tool_option_core_EXTERNAL_OBJECTS =

liblidar_tool_option_core.a: CMakeFiles/lidar_tool_option_core.dir/src/Lidar_Tool_Option/Lidar_Tool_Option.cpp.o
liblidar_tool_option_core.a: CMakeFiles/lidar_tool_option_core.dir/build.make
liblidar_tool_option_core.a: CMakeFiles/lidar_tool_option_core.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ahmet/Lidar_Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblidar_tool_option_core.a"
	$(CMAKE_COMMAND) -P CMakeFiles/lidar_tool_option_core.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidar_tool_option_core.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lidar_tool_option_core.dir/build: liblidar_tool_option_core.a

.PHONY : CMakeFiles/lidar_tool_option_core.dir/build

CMakeFiles/lidar_tool_option_core.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lidar_tool_option_core.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lidar_tool_option_core.dir/clean

CMakeFiles/lidar_tool_option_core.dir/depend:
	cd /home/ahmet/Lidar_Simulation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ahmet/Lidar_Simulation /home/ahmet/Lidar_Simulation /home/ahmet/Lidar_Simulation/build /home/ahmet/Lidar_Simulation/build /home/ahmet/Lidar_Simulation/build/CMakeFiles/lidar_tool_option_core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lidar_tool_option_core.dir/depend

