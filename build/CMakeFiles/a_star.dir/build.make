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
CMAKE_SOURCE_DIR = /home/hadar/Project/libMultiRobotPlanning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hadar/Project/libMultiRobotPlanning/build

# Include any dependencies generated for this target.
include CMakeFiles/a_star.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/a_star.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/a_star.dir/flags.make

CMakeFiles/a_star.dir/example/a_star.cpp.o: CMakeFiles/a_star.dir/flags.make
CMakeFiles/a_star.dir/example/a_star.cpp.o: ../example/a_star.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hadar/Project/libMultiRobotPlanning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/a_star.dir/example/a_star.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a_star.dir/example/a_star.cpp.o -c /home/hadar/Project/libMultiRobotPlanning/example/a_star.cpp

CMakeFiles/a_star.dir/example/a_star.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a_star.dir/example/a_star.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hadar/Project/libMultiRobotPlanning/example/a_star.cpp > CMakeFiles/a_star.dir/example/a_star.cpp.i

CMakeFiles/a_star.dir/example/a_star.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a_star.dir/example/a_star.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hadar/Project/libMultiRobotPlanning/example/a_star.cpp -o CMakeFiles/a_star.dir/example/a_star.cpp.s

CMakeFiles/a_star.dir/example/a_star.cpp.o.requires:

.PHONY : CMakeFiles/a_star.dir/example/a_star.cpp.o.requires

CMakeFiles/a_star.dir/example/a_star.cpp.o.provides: CMakeFiles/a_star.dir/example/a_star.cpp.o.requires
	$(MAKE) -f CMakeFiles/a_star.dir/build.make CMakeFiles/a_star.dir/example/a_star.cpp.o.provides.build
.PHONY : CMakeFiles/a_star.dir/example/a_star.cpp.o.provides

CMakeFiles/a_star.dir/example/a_star.cpp.o.provides.build: CMakeFiles/a_star.dir/example/a_star.cpp.o


# Object files for target a_star
a_star_OBJECTS = \
"CMakeFiles/a_star.dir/example/a_star.cpp.o"

# External object files for target a_star
a_star_EXTERNAL_OBJECTS =

a_star: CMakeFiles/a_star.dir/example/a_star.cpp.o
a_star: CMakeFiles/a_star.dir/build.make
a_star: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
a_star: CMakeFiles/a_star.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hadar/Project/libMultiRobotPlanning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable a_star"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a_star.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/a_star.dir/build: a_star

.PHONY : CMakeFiles/a_star.dir/build

CMakeFiles/a_star.dir/requires: CMakeFiles/a_star.dir/example/a_star.cpp.o.requires

.PHONY : CMakeFiles/a_star.dir/requires

CMakeFiles/a_star.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/a_star.dir/cmake_clean.cmake
.PHONY : CMakeFiles/a_star.dir/clean

CMakeFiles/a_star.dir/depend:
	cd /home/hadar/Project/libMultiRobotPlanning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hadar/Project/libMultiRobotPlanning /home/hadar/Project/libMultiRobotPlanning /home/hadar/Project/libMultiRobotPlanning/build /home/hadar/Project/libMultiRobotPlanning/build /home/hadar/Project/libMultiRobotPlanning/build/CMakeFiles/a_star.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/a_star.dir/depend

