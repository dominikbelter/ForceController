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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/maciej/Pulpit/mgr/ForceController

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maciej/Pulpit/mgr/ForceController/build

# Include any dependencies generated for this target.
include CMakeFiles/Kinematic.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Kinematic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Kinematic.dir/flags.make

CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o: CMakeFiles/Kinematic.dir/flags.make
CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o: ../src/kinematic/kinematicLie.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/maciej/Pulpit/mgr/ForceController/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o -c /home/maciej/Pulpit/mgr/ForceController/src/kinematic/kinematicLie.cpp

CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/maciej/Pulpit/mgr/ForceController/src/kinematic/kinematicLie.cpp > CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.i

CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/maciej/Pulpit/mgr/ForceController/src/kinematic/kinematicLie.cpp -o CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.s

CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.requires:
.PHONY : CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.requires

CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.provides: CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.requires
	$(MAKE) -f CMakeFiles/Kinematic.dir/build.make CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.provides.build
.PHONY : CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.provides

CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.provides.build: CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o

# Object files for target Kinematic
Kinematic_OBJECTS = \
"CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o"

# External object files for target Kinematic
Kinematic_EXTERNAL_OBJECTS =

lib/libKinematic.a: CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o
lib/libKinematic.a: CMakeFiles/Kinematic.dir/build.make
lib/libKinematic.a: CMakeFiles/Kinematic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library lib/libKinematic.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Kinematic.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Kinematic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Kinematic.dir/build: lib/libKinematic.a
.PHONY : CMakeFiles/Kinematic.dir/build

CMakeFiles/Kinematic.dir/requires: CMakeFiles/Kinematic.dir/src/kinematic/kinematicLie.cpp.o.requires
.PHONY : CMakeFiles/Kinematic.dir/requires

CMakeFiles/Kinematic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Kinematic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Kinematic.dir/clean

CMakeFiles/Kinematic.dir/depend:
	cd /home/maciej/Pulpit/mgr/ForceController/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maciej/Pulpit/mgr/ForceController /home/maciej/Pulpit/mgr/ForceController /home/maciej/Pulpit/mgr/ForceController/build /home/maciej/Pulpit/mgr/ForceController/build /home/maciej/Pulpit/mgr/ForceController/build/CMakeFiles/Kinematic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Kinematic.dir/depend

