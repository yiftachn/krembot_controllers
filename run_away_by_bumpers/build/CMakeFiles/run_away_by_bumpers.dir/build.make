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
CMAKE_SOURCE_DIR = /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/build

# Include any dependencies generated for this target.
include CMakeFiles/run_away_by_bumpers.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run_away_by_bumpers.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_away_by_bumpers.dir/flags.make

CMakeFiles/run_away_by_bumpers.dir/krembot.ino.o: CMakeFiles/run_away_by_bumpers.dir/flags.make
CMakeFiles/run_away_by_bumpers.dir/krembot.ino.o: ../krembot.ino.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_away_by_bumpers.dir/krembot.ino.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_away_by_bumpers.dir/krembot.ino.o -c /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/krembot.ino.cpp

CMakeFiles/run_away_by_bumpers.dir/krembot.ino.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_away_by_bumpers.dir/krembot.ino.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/krembot.ino.cpp > CMakeFiles/run_away_by_bumpers.dir/krembot.ino.i

CMakeFiles/run_away_by_bumpers.dir/krembot.ino.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_away_by_bumpers.dir/krembot.ino.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/krembot.ino.cpp -o CMakeFiles/run_away_by_bumpers.dir/krembot.ino.s

# Object files for target run_away_by_bumpers
run_away_by_bumpers_OBJECTS = \
"CMakeFiles/run_away_by_bumpers.dir/krembot.ino.o"

# External object files for target run_away_by_bumpers
run_away_by_bumpers_EXTERNAL_OBJECTS =

librun_away_by_bumpers.so: CMakeFiles/run_away_by_bumpers.dir/krembot.ino.o
librun_away_by_bumpers.so: CMakeFiles/run_away_by_bumpers.dir/build.make
librun_away_by_bumpers.so: CMakeFiles/run_away_by_bumpers.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module librun_away_by_bumpers.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_away_by_bumpers.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run_away_by_bumpers.dir/build: librun_away_by_bumpers.so

.PHONY : CMakeFiles/run_away_by_bumpers.dir/build

CMakeFiles/run_away_by_bumpers.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_away_by_bumpers.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_away_by_bumpers.dir/clean

CMakeFiles/run_away_by_bumpers.dir/depend:
	cd /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/build /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/build /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/run_away_by_bumpers/build/CMakeFiles/run_away_by_bumpers.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_away_by_bumpers.dir/depend

