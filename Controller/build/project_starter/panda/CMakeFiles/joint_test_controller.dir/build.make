# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sunny/Documents/Haptic-EMT-Robot/Controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sunny/Documents/Haptic-EMT-Robot/Controller/build

# Include any dependencies generated for this target.
include project_starter/panda/CMakeFiles/joint_test_controller.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include project_starter/panda/CMakeFiles/joint_test_controller.dir/compiler_depend.make

# Include the progress variables for this target.
include project_starter/panda/CMakeFiles/joint_test_controller.dir/progress.make

# Include the compile flags for this target's objects.
include project_starter/panda/CMakeFiles/joint_test_controller.dir/flags.make

project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o: project_starter/panda/CMakeFiles/joint_test_controller.dir/flags.make
project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o: /home/sunny/Documents/Haptic-EMT-Robot/Controller/project_starter/panda/controller_joint_test.cpp
project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o: project_starter/panda/CMakeFiles/joint_test_controller.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/sunny/Documents/Haptic-EMT-Robot/Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o"
	cd /home/sunny/Documents/Haptic-EMT-Robot/Controller/build/project_starter/panda && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o -MF CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o.d -o CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o -c /home/sunny/Documents/Haptic-EMT-Robot/Controller/project_starter/panda/controller_joint_test.cpp

project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.i"
	cd /home/sunny/Documents/Haptic-EMT-Robot/Controller/build/project_starter/panda && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sunny/Documents/Haptic-EMT-Robot/Controller/project_starter/panda/controller_joint_test.cpp > CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.i

project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.s"
	cd /home/sunny/Documents/Haptic-EMT-Robot/Controller/build/project_starter/panda && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sunny/Documents/Haptic-EMT-Robot/Controller/project_starter/panda/controller_joint_test.cpp -o CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.s

# Object files for target joint_test_controller
joint_test_controller_OBJECTS = \
"CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o"

# External object files for target joint_test_controller
joint_test_controller_EXTERNAL_OBJECTS =

/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: project_starter/panda/CMakeFiles/joint_test_controller.dir/controller_joint_test.cpp.o
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: project_starter/panda/CMakeFiles/joint_test_controller.dir/build.make
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-interfaces/build/libsai2-interfaces.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-primitives/build/libsai2-primitives.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-primitives/ruckig/build/libruckig.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-simulation/build/libsai2-simulation.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-simulation/lib/linux/x86_64/libsai2-simulation-core.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-graphics/build/libsai2-graphics.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libglfw.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/chai3d/build/libchai3d.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libOpenGL.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libGLX.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libopenal.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-model/build/libsai2-model.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-model/rbdl/build/librbdl.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-urdfreader/build/libsai2-urdf.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-common/build/libsai2-common.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libglfw.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-model/build/libsai2-model.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-model/rbdl/build/librbdl.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-graphics/build/libsai2-graphics.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libglfw.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-simulation/build/libsai2-simulation.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-simulation/lib/linux/x86_64/libsai2-simulation-core.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-common/build/libsai2-common.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-urdfreader/build/libsai2-urdf.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/chai3d/build/libchai3d.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libOpenGL.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libGLX.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libopenal.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-primitives/build/libsai2-primitives.a
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: /home/sunny/Documents/OpenSai/core/sai2-primitives/ruckig/build/libruckig.so
/home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller: project_starter/panda/CMakeFiles/joint_test_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/sunny/Documents/Haptic-EMT-Robot/Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller"
	cd /home/sunny/Documents/Haptic-EMT-Robot/Controller/build/project_starter/panda && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joint_test_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
project_starter/panda/CMakeFiles/joint_test_controller.dir/build: /home/sunny/Documents/Haptic-EMT-Robot/Controller/bin/joint_test_controller
.PHONY : project_starter/panda/CMakeFiles/joint_test_controller.dir/build

project_starter/panda/CMakeFiles/joint_test_controller.dir/clean:
	cd /home/sunny/Documents/Haptic-EMT-Robot/Controller/build/project_starter/panda && $(CMAKE_COMMAND) -P CMakeFiles/joint_test_controller.dir/cmake_clean.cmake
.PHONY : project_starter/panda/CMakeFiles/joint_test_controller.dir/clean

project_starter/panda/CMakeFiles/joint_test_controller.dir/depend:
	cd /home/sunny/Documents/Haptic-EMT-Robot/Controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sunny/Documents/Haptic-EMT-Robot/Controller /home/sunny/Documents/Haptic-EMT-Robot/Controller/project_starter/panda /home/sunny/Documents/Haptic-EMT-Robot/Controller/build /home/sunny/Documents/Haptic-EMT-Robot/Controller/build/project_starter/panda /home/sunny/Documents/Haptic-EMT-Robot/Controller/build/project_starter/panda/CMakeFiles/joint_test_controller.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : project_starter/panda/CMakeFiles/joint_test_controller.dir/depend

