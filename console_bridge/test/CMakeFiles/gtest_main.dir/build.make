# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/lorenzo/mbzirc2020_ws/src/console_bridge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lorenzo/mbzirc2020_ws/src/console_bridge

# Include any dependencies generated for this target.
include test/CMakeFiles/gtest_main.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/gtest_main.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/gtest_main.dir/flags.make

test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o: test/CMakeFiles/gtest_main.dir/flags.make
test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o: test/gtest/src/gtest_main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lorenzo/mbzirc2020_ws/src/console_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o"
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o -c /home/lorenzo/mbzirc2020_ws/src/console_bridge/test/gtest/src/gtest_main.cc

test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.i"
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lorenzo/mbzirc2020_ws/src/console_bridge/test/gtest/src/gtest_main.cc > CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.i

test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.s"
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lorenzo/mbzirc2020_ws/src/console_bridge/test/gtest/src/gtest_main.cc -o CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.s

test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.requires:

.PHONY : test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.requires

test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.provides: test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.requires
	$(MAKE) -f test/CMakeFiles/gtest_main.dir/build.make test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.provides.build
.PHONY : test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.provides

test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.provides.build: test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o


# Object files for target gtest_main
gtest_main_OBJECTS = \
"CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o"

# External object files for target gtest_main
gtest_main_EXTERNAL_OBJECTS =

lib/libgtest_main.a: test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o
lib/libgtest_main.a: test/CMakeFiles/gtest_main.dir/build.make
lib/libgtest_main.a: test/CMakeFiles/gtest_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lorenzo/mbzirc2020_ws/src/console_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../lib/libgtest_main.a"
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge/test && $(CMAKE_COMMAND) -P CMakeFiles/gtest_main.dir/cmake_clean_target.cmake
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtest_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/gtest_main.dir/build: lib/libgtest_main.a

.PHONY : test/CMakeFiles/gtest_main.dir/build

test/CMakeFiles/gtest_main.dir/requires: test/CMakeFiles/gtest_main.dir/gtest/src/gtest_main.cc.o.requires

.PHONY : test/CMakeFiles/gtest_main.dir/requires

test/CMakeFiles/gtest_main.dir/clean:
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge/test && $(CMAKE_COMMAND) -P CMakeFiles/gtest_main.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/gtest_main.dir/clean

test/CMakeFiles/gtest_main.dir/depend:
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lorenzo/mbzirc2020_ws/src/console_bridge /home/lorenzo/mbzirc2020_ws/src/console_bridge/test /home/lorenzo/mbzirc2020_ws/src/console_bridge /home/lorenzo/mbzirc2020_ws/src/console_bridge/test /home/lorenzo/mbzirc2020_ws/src/console_bridge/test/CMakeFiles/gtest_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/gtest_main.dir/depend
