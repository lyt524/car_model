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
include CMakeFiles/CppTest.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/CppTest.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/CppTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CppTest.dir/flags.make

CMakeFiles/CppTest.dir/test.cpp.o: CMakeFiles/CppTest.dir/flags.make
CMakeFiles/CppTest.dir/test.cpp.o: ../test.cpp
CMakeFiles/CppTest.dir/test.cpp.o: CMakeFiles/CppTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CppTest.dir/test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/CppTest.dir/test.cpp.o -MF CMakeFiles/CppTest.dir/test.cpp.o.d -o CMakeFiles/CppTest.dir/test.cpp.o -c /home/plusai/car_model/test.cpp

CMakeFiles/CppTest.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CppTest.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/test.cpp > CMakeFiles/CppTest.dir/test.cpp.i

CMakeFiles/CppTest.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CppTest.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/test.cpp -o CMakeFiles/CppTest.dir/test.cpp.s

CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o: CMakeFiles/CppTest.dir/flags.make
CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o: ../models/kinematics_model.cpp
CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o: CMakeFiles/CppTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o -MF CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o.d -o CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o -c /home/plusai/car_model/models/kinematics_model.cpp

CMakeFiles/CppTest.dir/models/kinematics_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CppTest.dir/models/kinematics_model.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/models/kinematics_model.cpp > CMakeFiles/CppTest.dir/models/kinematics_model.cpp.i

CMakeFiles/CppTest.dir/models/kinematics_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CppTest.dir/models/kinematics_model.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/models/kinematics_model.cpp -o CMakeFiles/CppTest.dir/models/kinematics_model.cpp.s

CMakeFiles/CppTest.dir/tools/mathtools.cpp.o: CMakeFiles/CppTest.dir/flags.make
CMakeFiles/CppTest.dir/tools/mathtools.cpp.o: ../tools/mathtools.cpp
CMakeFiles/CppTest.dir/tools/mathtools.cpp.o: CMakeFiles/CppTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/CppTest.dir/tools/mathtools.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/CppTest.dir/tools/mathtools.cpp.o -MF CMakeFiles/CppTest.dir/tools/mathtools.cpp.o.d -o CMakeFiles/CppTest.dir/tools/mathtools.cpp.o -c /home/plusai/car_model/tools/mathtools.cpp

CMakeFiles/CppTest.dir/tools/mathtools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CppTest.dir/tools/mathtools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/tools/mathtools.cpp > CMakeFiles/CppTest.dir/tools/mathtools.cpp.i

CMakeFiles/CppTest.dir/tools/mathtools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CppTest.dir/tools/mathtools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/tools/mathtools.cpp -o CMakeFiles/CppTest.dir/tools/mathtools.cpp.s

CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o: CMakeFiles/CppTest.dir/flags.make
CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o: ../referencepath/reference_path.cpp
CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o: CMakeFiles/CppTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o -MF CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o.d -o CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o -c /home/plusai/car_model/referencepath/reference_path.cpp

CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/referencepath/reference_path.cpp > CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.i

CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/referencepath/reference_path.cpp -o CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.s

# Object files for target CppTest
CppTest_OBJECTS = \
"CMakeFiles/CppTest.dir/test.cpp.o" \
"CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o" \
"CMakeFiles/CppTest.dir/tools/mathtools.cpp.o" \
"CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o"

# External object files for target CppTest
CppTest_EXTERNAL_OBJECTS =

CppTest: CMakeFiles/CppTest.dir/test.cpp.o
CppTest: CMakeFiles/CppTest.dir/models/kinematics_model.cpp.o
CppTest: CMakeFiles/CppTest.dir/tools/mathtools.cpp.o
CppTest: CMakeFiles/CppTest.dir/referencepath/reference_path.cpp.o
CppTest: CMakeFiles/CppTest.dir/build.make
CppTest: CMakeFiles/CppTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable CppTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CppTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CppTest.dir/build: CppTest
.PHONY : CMakeFiles/CppTest.dir/build

CMakeFiles/CppTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CppTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CppTest.dir/clean

CMakeFiles/CppTest.dir/depend:
	cd /home/plusai/car_model/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/plusai/car_model /home/plusai/car_model /home/plusai/car_model/build /home/plusai/car_model/build /home/plusai/car_model/build/CMakeFiles/CppTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CppTest.dir/depend

