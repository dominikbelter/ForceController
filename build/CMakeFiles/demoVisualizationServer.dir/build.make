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
include CMakeFiles/demoVisualizationServer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/demoVisualizationServer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demoVisualizationServer.dir/flags.make

CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o: CMakeFiles/demoVisualizationServer.dir/flags.make
CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o: ../demoVisualizationServer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/maciej/Pulpit/mgr/ForceController/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o -c /home/maciej/Pulpit/mgr/ForceController/demoVisualizationServer.cpp

CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/maciej/Pulpit/mgr/ForceController/demoVisualizationServer.cpp > CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.i

CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/maciej/Pulpit/mgr/ForceController/demoVisualizationServer.cpp -o CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.s

CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.requires:
.PHONY : CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.requires

CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.provides: CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.requires
	$(MAKE) -f CMakeFiles/demoVisualizationServer.dir/build.make CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.provides.build
.PHONY : CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.provides

CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.provides.build: CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o

# Object files for target demoVisualizationServer
demoVisualizationServer_OBJECTS = \
"CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o"

# External object files for target demoVisualizationServer
demoVisualizationServer_EXTERNAL_OBJECTS =

bin/demoVisualizationServer: CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o
bin/demoVisualizationServer: CMakeFiles/demoVisualizationServer.dir/build.make
bin/demoVisualizationServer: lib/libLeg.a
bin/demoVisualizationServer: lib/libBoard.a
bin/demoVisualizationServer: lib/libRobot.a
bin/demoVisualizationServer: lib/libVisualization.a
bin/demoVisualizationServer: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/demoVisualizationServer: lib/libdynamixel.a
bin/demoVisualizationServer: lib/libRobot.a
bin/demoVisualizationServer: lib/libLeg.a
bin/demoVisualizationServer: lib/libKinematic.a
bin/demoVisualizationServer: lib/libtinyxml2.a
bin/demoVisualizationServer: CMakeFiles/demoVisualizationServer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/demoVisualizationServer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demoVisualizationServer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demoVisualizationServer.dir/build: bin/demoVisualizationServer
.PHONY : CMakeFiles/demoVisualizationServer.dir/build

CMakeFiles/demoVisualizationServer.dir/requires: CMakeFiles/demoVisualizationServer.dir/demoVisualizationServer.cpp.o.requires
.PHONY : CMakeFiles/demoVisualizationServer.dir/requires

CMakeFiles/demoVisualizationServer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demoVisualizationServer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demoVisualizationServer.dir/clean

CMakeFiles/demoVisualizationServer.dir/depend:
	cd /home/maciej/Pulpit/mgr/ForceController/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maciej/Pulpit/mgr/ForceController /home/maciej/Pulpit/mgr/ForceController /home/maciej/Pulpit/mgr/ForceController/build /home/maciej/Pulpit/mgr/ForceController/build /home/maciej/Pulpit/mgr/ForceController/build/CMakeFiles/demoVisualizationServer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demoVisualizationServer.dir/depend

