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
CMAKE_SOURCE_DIR = /home/plusai/car_model

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/plusai/car_model/build

# Include any dependencies generated for this target.
include CMakeFiles/KinematicsModelTest.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/KinematicsModelTest.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/KinematicsModelTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/KinematicsModelTest.dir/flags.make

CMakeFiles/KinematicsModelTest.dir/test.cpp.o: CMakeFiles/KinematicsModelTest.dir/flags.make
CMakeFiles/KinematicsModelTest.dir/test.cpp.o: ../test.cpp
CMakeFiles/KinematicsModelTest.dir/test.cpp.o: CMakeFiles/KinematicsModelTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/KinematicsModelTest.dir/test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/KinematicsModelTest.dir/test.cpp.o -MF CMakeFiles/KinematicsModelTest.dir/test.cpp.o.d -o CMakeFiles/KinematicsModelTest.dir/test.cpp.o -c /home/plusai/car_model/test.cpp

CMakeFiles/KinematicsModelTest.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KinematicsModelTest.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/test.cpp > CMakeFiles/KinematicsModelTest.dir/test.cpp.i

CMakeFiles/KinematicsModelTest.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KinematicsModelTest.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/test.cpp -o CMakeFiles/KinematicsModelTest.dir/test.cpp.s

CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o: CMakeFiles/KinematicsModelTest.dir/flags.make
CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o: ../models/kinematics_model.cpp
CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o: CMakeFiles/KinematicsModelTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o -MF CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o.d -o CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o -c /home/plusai/car_model/models/kinematics_model.cpp

CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/models/kinematics_model.cpp > CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.i

CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/models/kinematics_model.cpp -o CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.s

CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o: CMakeFiles/KinematicsModelTest.dir/flags.make
CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o: ../tools/mathtools.cpp
CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o: CMakeFiles/KinematicsModelTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o -MF CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o.d -o CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o -c /home/plusai/car_model/tools/mathtools.cpp

CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/tools/mathtools.cpp > CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.i

CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/tools/mathtools.cpp -o CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.s

# Object files for target KinematicsModelTest
KinematicsModelTest_OBJECTS = \
"CMakeFiles/KinematicsModelTest.dir/test.cpp.o" \
"CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o" \
"CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o"

# External object files for target KinematicsModelTest
KinematicsModelTest_EXTERNAL_OBJECTS =

KinematicsModelTest: CMakeFiles/KinematicsModelTest.dir/test.cpp.o
KinematicsModelTest: CMakeFiles/KinematicsModelTest.dir/models/kinematics_model.cpp.o
KinematicsModelTest: CMakeFiles/KinematicsModelTest.dir/tools/mathtools.cpp.o
KinematicsModelTest: CMakeFiles/KinematicsModelTest.dir/build.make
KinematicsModelTest: CMakeFiles/KinematicsModelTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable KinematicsModelTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/KinematicsModelTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/KinematicsModelTest.dir/build: KinematicsModelTest
.PHONY : CMakeFiles/KinematicsModelTest.dir/build

CMakeFiles/KinematicsModelTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/KinematicsModelTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/KinematicsModelTest.dir/clean

CMakeFiles/KinematicsModelTest.dir/depend:
	cd /home/plusai/car_model/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/plusai/car_model /home/plusai/car_model /home/plusai/car_model/build /home/plusai/car_model/build /home/plusai/car_model/build/CMakeFiles/KinematicsModelTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/KinematicsModelTest.dir/depend

