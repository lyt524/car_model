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
include CMakeFiles/mpcTest.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mpcTest.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mpcTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mpcTest.dir/flags.make

CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o: CMakeFiles/mpcTest.dir/flags.make
CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o: ../test_mpc_1231.cpp
CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o: CMakeFiles/mpcTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o -MF CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o.d -o CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o -c /home/plusai/car_model/test_mpc_1231.cpp

CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/test_mpc_1231.cpp > CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.i

CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/test_mpc_1231.cpp -o CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.s

CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o: CMakeFiles/mpcTest.dir/flags.make
CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o: ../models/kinematics_model.cpp
CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o: CMakeFiles/mpcTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o -MF CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o.d -o CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o -c /home/plusai/car_model/models/kinematics_model.cpp

CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/models/kinematics_model.cpp > CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.i

CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/models/kinematics_model.cpp -o CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.s

CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o: CMakeFiles/mpcTest.dir/flags.make
CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o: ../models/bicycle_model_mpc.cpp
CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o: CMakeFiles/mpcTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o -MF CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o.d -o CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o -c /home/plusai/car_model/models/bicycle_model_mpc.cpp

CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/models/bicycle_model_mpc.cpp > CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.i

CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/models/bicycle_model_mpc.cpp -o CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.s

CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o: CMakeFiles/mpcTest.dir/flags.make
CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o: ../tools/mathtools.cpp
CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o: CMakeFiles/mpcTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o -MF CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o.d -o CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o -c /home/plusai/car_model/tools/mathtools.cpp

CMakeFiles/mpcTest.dir/tools/mathtools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpcTest.dir/tools/mathtools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/tools/mathtools.cpp > CMakeFiles/mpcTest.dir/tools/mathtools.cpp.i

CMakeFiles/mpcTest.dir/tools/mathtools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpcTest.dir/tools/mathtools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/tools/mathtools.cpp -o CMakeFiles/mpcTest.dir/tools/mathtools.cpp.s

CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o: CMakeFiles/mpcTest.dir/flags.make
CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o: ../referencepath/reference_path.cpp
CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o: CMakeFiles/mpcTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o -MF CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o.d -o CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o -c /home/plusai/car_model/referencepath/reference_path.cpp

CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/referencepath/reference_path.cpp > CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.i

CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/referencepath/reference_path.cpp -o CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.s

CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o: CMakeFiles/mpcTest.dir/flags.make
CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o: ../control/mpc_coupling.cpp
CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o: CMakeFiles/mpcTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o -MF CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o.d -o CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o -c /home/plusai/car_model/control/mpc_coupling.cpp

CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/control/mpc_coupling.cpp > CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.i

CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/control/mpc_coupling.cpp -o CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.s

# Object files for target mpcTest
mpcTest_OBJECTS = \
"CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o" \
"CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o" \
"CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o" \
"CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o" \
"CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o" \
"CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o"

# External object files for target mpcTest
mpcTest_EXTERNAL_OBJECTS =

mpcTest: CMakeFiles/mpcTest.dir/test_mpc_1231.cpp.o
mpcTest: CMakeFiles/mpcTest.dir/models/kinematics_model.cpp.o
mpcTest: CMakeFiles/mpcTest.dir/models/bicycle_model_mpc.cpp.o
mpcTest: CMakeFiles/mpcTest.dir/tools/mathtools.cpp.o
mpcTest: CMakeFiles/mpcTest.dir/referencepath/reference_path.cpp.o
mpcTest: CMakeFiles/mpcTest.dir/control/mpc_coupling.cpp.o
mpcTest: CMakeFiles/mpcTest.dir/build.make
mpcTest: /usr/local/lib/libOsqpEigen.so.0.8.1
mpcTest: /usr/local/lib/libosqp.so
mpcTest: CMakeFiles/mpcTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable mpcTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpcTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mpcTest.dir/build: mpcTest
.PHONY : CMakeFiles/mpcTest.dir/build

CMakeFiles/mpcTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mpcTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mpcTest.dir/clean

CMakeFiles/mpcTest.dir/depend:
	cd /home/plusai/car_model/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/plusai/car_model /home/plusai/car_model /home/plusai/car_model/build /home/plusai/car_model/build /home/plusai/car_model/build/CMakeFiles/mpcTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mpcTest.dir/depend
