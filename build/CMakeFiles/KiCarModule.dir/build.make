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
include CMakeFiles/KiCarModule.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/KiCarModule.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/KiCarModule.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/KiCarModule.dir/flags.make

CMakeFiles/KiCarModule.dir/bind.cpp.o: CMakeFiles/KiCarModule.dir/flags.make
CMakeFiles/KiCarModule.dir/bind.cpp.o: ../bind.cpp
CMakeFiles/KiCarModule.dir/bind.cpp.o: CMakeFiles/KiCarModule.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/KiCarModule.dir/bind.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/KiCarModule.dir/bind.cpp.o -MF CMakeFiles/KiCarModule.dir/bind.cpp.o.d -o CMakeFiles/KiCarModule.dir/bind.cpp.o -c /home/plusai/car_model/bind.cpp

CMakeFiles/KiCarModule.dir/bind.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KiCarModule.dir/bind.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/bind.cpp > CMakeFiles/KiCarModule.dir/bind.cpp.i

CMakeFiles/KiCarModule.dir/bind.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KiCarModule.dir/bind.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/bind.cpp -o CMakeFiles/KiCarModule.dir/bind.cpp.s

CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o: CMakeFiles/KiCarModule.dir/flags.make
CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o: ../kinematics_model.cpp
CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o: CMakeFiles/KiCarModule.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o -MF CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o.d -o CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o -c /home/plusai/car_model/kinematics_model.cpp

CMakeFiles/KiCarModule.dir/kinematics_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KiCarModule.dir/kinematics_model.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/plusai/car_model/kinematics_model.cpp > CMakeFiles/KiCarModule.dir/kinematics_model.cpp.i

CMakeFiles/KiCarModule.dir/kinematics_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KiCarModule.dir/kinematics_model.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/plusai/car_model/kinematics_model.cpp -o CMakeFiles/KiCarModule.dir/kinematics_model.cpp.s

# Object files for target KiCarModule
KiCarModule_OBJECTS = \
"CMakeFiles/KiCarModule.dir/bind.cpp.o" \
"CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o"

# External object files for target KiCarModule
KiCarModule_EXTERNAL_OBJECTS =

KiCarModule.cpython-310-x86_64-linux-gnu.so: CMakeFiles/KiCarModule.dir/bind.cpp.o
KiCarModule.cpython-310-x86_64-linux-gnu.so: CMakeFiles/KiCarModule.dir/kinematics_model.cpp.o
KiCarModule.cpython-310-x86_64-linux-gnu.so: CMakeFiles/KiCarModule.dir/build.make
KiCarModule.cpython-310-x86_64-linux-gnu.so: CMakeFiles/KiCarModule.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/plusai/car_model/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module KiCarModule.cpython-310-x86_64-linux-gnu.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/KiCarModule.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/strip /home/plusai/car_model/build/KiCarModule.cpython-310-x86_64-linux-gnu.so

# Rule to build all files generated by this target.
CMakeFiles/KiCarModule.dir/build: KiCarModule.cpython-310-x86_64-linux-gnu.so
.PHONY : CMakeFiles/KiCarModule.dir/build

CMakeFiles/KiCarModule.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/KiCarModule.dir/cmake_clean.cmake
.PHONY : CMakeFiles/KiCarModule.dir/clean

CMakeFiles/KiCarModule.dir/depend:
	cd /home/plusai/car_model/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/plusai/car_model /home/plusai/car_model /home/plusai/car_model/build /home/plusai/car_model/build /home/plusai/car_model/build/CMakeFiles/KiCarModule.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/KiCarModule.dir/depend
