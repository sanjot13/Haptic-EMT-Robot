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
CMAKE_SOURCE_DIR = /home/sunny/Documents/cs225a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sunny/Documents/cs225a/build

# Include any dependencies generated for this target.
include homework/hw2/CMakeFiles/hw2-simviz.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include homework/hw2/CMakeFiles/hw2-simviz.dir/compiler_depend.make

# Include the progress variables for this target.
include homework/hw2/CMakeFiles/hw2-simviz.dir/progress.make

# Include the compile flags for this target's objects.
include homework/hw2/CMakeFiles/hw2-simviz.dir/flags.make

homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o: homework/hw2/CMakeFiles/hw2-simviz.dir/flags.make
homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o: /home/sunny/Documents/cs225a/homework/hw2/hw2-simviz.cpp
homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o: homework/hw2/CMakeFiles/hw2-simviz.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/sunny/Documents/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o"
	cd /home/sunny/Documents/cs225a/build/homework/hw2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o -MF CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o.d -o CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o -c /home/sunny/Documents/cs225a/homework/hw2/hw2-simviz.cpp

homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.i"
	cd /home/sunny/Documents/cs225a/build/homework/hw2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sunny/Documents/cs225a/homework/hw2/hw2-simviz.cpp > CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.i

homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.s"
	cd /home/sunny/Documents/cs225a/build/homework/hw2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sunny/Documents/cs225a/homework/hw2/hw2-simviz.cpp -o CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.s

# Object files for target hw2-simviz
hw2__simviz_OBJECTS = \
"CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o"

# External object files for target hw2-simviz
hw2__simviz_EXTERNAL_OBJECTS =

/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: homework/hw2/CMakeFiles/hw2-simviz.dir/hw2-simviz.cpp.o
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: homework/hw2/CMakeFiles/hw2-simviz.dir/build.make
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-interfaces/build/libsai2-interfaces.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-primitives/build/libsai2-primitives.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-primitives/ruckig/build/libruckig.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-simulation/build/libsai2-simulation.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-simulation/lib/linux/x86_64/libsai2-simulation-core.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-graphics/build/libsai2-graphics.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libglfw.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/chai3d/build/libchai3d.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libOpenGL.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libGLX.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libopenal.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-model/build/libsai2-model.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-model/rbdl/build/librbdl.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-urdfreader/build/libsai2-urdf.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-common/build/libsai2-common.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libglfw.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-model/build/libsai2-model.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-model/rbdl/build/librbdl.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-graphics/build/libsai2-graphics.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libglfw.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-simulation/build/libsai2-simulation.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-simulation/lib/linux/x86_64/libsai2-simulation-core.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-common/build/libsai2-common.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-urdfreader/build/libsai2-urdf.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/chai3d/build/libchai3d.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libOpenGL.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libGLX.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libopenal.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /usr/lib/x86_64-linux-gnu/libhiredis.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-primitives/build/libsai2-primitives.a
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: /home/sunny/Documents/OpenSai/core/sai2-primitives/ruckig/build/libruckig.so
/home/sunny/Documents/cs225a/bin/hw2/hw2-simviz: homework/hw2/CMakeFiles/hw2-simviz.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/sunny/Documents/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/sunny/Documents/cs225a/bin/hw2/hw2-simviz"
	cd /home/sunny/Documents/cs225a/build/homework/hw2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hw2-simviz.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
homework/hw2/CMakeFiles/hw2-simviz.dir/build: /home/sunny/Documents/cs225a/bin/hw2/hw2-simviz
.PHONY : homework/hw2/CMakeFiles/hw2-simviz.dir/build

homework/hw2/CMakeFiles/hw2-simviz.dir/clean:
	cd /home/sunny/Documents/cs225a/build/homework/hw2 && $(CMAKE_COMMAND) -P CMakeFiles/hw2-simviz.dir/cmake_clean.cmake
.PHONY : homework/hw2/CMakeFiles/hw2-simviz.dir/clean

homework/hw2/CMakeFiles/hw2-simviz.dir/depend:
	cd /home/sunny/Documents/cs225a/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sunny/Documents/cs225a /home/sunny/Documents/cs225a/homework/hw2 /home/sunny/Documents/cs225a/build /home/sunny/Documents/cs225a/build/homework/hw2 /home/sunny/Documents/cs225a/build/homework/hw2/CMakeFiles/hw2-simviz.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : homework/hw2/CMakeFiles/hw2-simviz.dir/depend
