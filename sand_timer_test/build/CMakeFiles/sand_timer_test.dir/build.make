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
CMAKE_SOURCE_DIR = /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/build

# Include any dependencies generated for this target.
include CMakeFiles/sand_timer_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sand_timer_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sand_timer_test.dir/flags.make

CMakeFiles/sand_timer_test.dir/krembot.ino.o: CMakeFiles/sand_timer_test.dir/flags.make
CMakeFiles/sand_timer_test.dir/krembot.ino.o: ../krembot.ino.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sand_timer_test.dir/krembot.ino.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sand_timer_test.dir/krembot.ino.o -c /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/krembot.ino.cpp

CMakeFiles/sand_timer_test.dir/krembot.ino.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sand_timer_test.dir/krembot.ino.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/krembot.ino.cpp > CMakeFiles/sand_timer_test.dir/krembot.ino.i

CMakeFiles/sand_timer_test.dir/krembot.ino.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sand_timer_test.dir/krembot.ino.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/krembot.ino.cpp -o CMakeFiles/sand_timer_test.dir/krembot.ino.s

# Object files for target sand_timer_test
sand_timer_test_OBJECTS = \
"CMakeFiles/sand_timer_test.dir/krembot.ino.o"

# External object files for target sand_timer_test
sand_timer_test_EXTERNAL_OBJECTS =

libsand_timer_test.so: CMakeFiles/sand_timer_test.dir/krembot.ino.o
libsand_timer_test.so: CMakeFiles/sand_timer_test.dir/build.make
libsand_timer_test.so: CMakeFiles/sand_timer_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module libsand_timer_test.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sand_timer_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sand_timer_test.dir/build: libsand_timer_test.so

.PHONY : CMakeFiles/sand_timer_test.dir/build

CMakeFiles/sand_timer_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sand_timer_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sand_timer_test.dir/clean

CMakeFiles/sand_timer_test.dir/depend:
	cd /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/build /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/build /home/yiftach/CLionProjects/krembot_sim/krembot_ws/controllers/sand_timer_test/build/CMakeFiles/sand_timer_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sand_timer_test.dir/depend

