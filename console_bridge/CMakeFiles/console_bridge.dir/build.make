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
include CMakeFiles/console_bridge.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/console_bridge.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/console_bridge.dir/flags.make

CMakeFiles/console_bridge.dir/src/console.cpp.o: CMakeFiles/console_bridge.dir/flags.make
CMakeFiles/console_bridge.dir/src/console.cpp.o: src/console.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lorenzo/mbzirc2020_ws/src/console_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/console_bridge.dir/src/console.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/console_bridge.dir/src/console.cpp.o -c /home/lorenzo/mbzirc2020_ws/src/console_bridge/src/console.cpp

CMakeFiles/console_bridge.dir/src/console.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/console_bridge.dir/src/console.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lorenzo/mbzirc2020_ws/src/console_bridge/src/console.cpp > CMakeFiles/console_bridge.dir/src/console.cpp.i

CMakeFiles/console_bridge.dir/src/console.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/console_bridge.dir/src/console.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lorenzo/mbzirc2020_ws/src/console_bridge/src/console.cpp -o CMakeFiles/console_bridge.dir/src/console.cpp.s

CMakeFiles/console_bridge.dir/src/console.cpp.o.requires:

.PHONY : CMakeFiles/console_bridge.dir/src/console.cpp.o.requires

CMakeFiles/console_bridge.dir/src/console.cpp.o.provides: CMakeFiles/console_bridge.dir/src/console.cpp.o.requires
	$(MAKE) -f CMakeFiles/console_bridge.dir/build.make CMakeFiles/console_bridge.dir/src/console.cpp.o.provides.build
.PHONY : CMakeFiles/console_bridge.dir/src/console.cpp.o.provides

CMakeFiles/console_bridge.dir/src/console.cpp.o.provides.build: CMakeFiles/console_bridge.dir/src/console.cpp.o


# Object files for target console_bridge
console_bridge_OBJECTS = \
"CMakeFiles/console_bridge.dir/src/console.cpp.o"

# External object files for target console_bridge
console_bridge_EXTERNAL_OBJECTS =

lib/libconsole_bridge.so.0.4: CMakeFiles/console_bridge.dir/src/console.cpp.o
lib/libconsole_bridge.so.0.4: CMakeFiles/console_bridge.dir/build.make
lib/libconsole_bridge.so.0.4: CMakeFiles/console_bridge.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lorenzo/mbzirc2020_ws/src/console_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library lib/libconsole_bridge.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/console_bridge.dir/link.txt --verbose=$(VERBOSE)
	$(CMAKE_COMMAND) -E cmake_symlink_library lib/libconsole_bridge.so.0.4 lib/libconsole_bridge.so.0.4 lib/libconsole_bridge.so

lib/libconsole_bridge.so: lib/libconsole_bridge.so.0.4
	@$(CMAKE_COMMAND) -E touch_nocreate lib/libconsole_bridge.so

# Rule to build all files generated by this target.
CMakeFiles/console_bridge.dir/build: lib/libconsole_bridge.so

.PHONY : CMakeFiles/console_bridge.dir/build

CMakeFiles/console_bridge.dir/requires: CMakeFiles/console_bridge.dir/src/console.cpp.o.requires

.PHONY : CMakeFiles/console_bridge.dir/requires

CMakeFiles/console_bridge.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/console_bridge.dir/cmake_clean.cmake
.PHONY : CMakeFiles/console_bridge.dir/clean

CMakeFiles/console_bridge.dir/depend:
	cd /home/lorenzo/mbzirc2020_ws/src/console_bridge && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lorenzo/mbzirc2020_ws/src/console_bridge /home/lorenzo/mbzirc2020_ws/src/console_bridge /home/lorenzo/mbzirc2020_ws/src/console_bridge /home/lorenzo/mbzirc2020_ws/src/console_bridge /home/lorenzo/mbzirc2020_ws/src/console_bridge/CMakeFiles/console_bridge.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/console_bridge.dir/depend

