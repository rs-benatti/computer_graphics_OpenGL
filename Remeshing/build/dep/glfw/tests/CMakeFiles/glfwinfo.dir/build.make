# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_SOURCE_DIR = /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build

# Include any dependencies generated for this target.
include dep/glfw/tests/CMakeFiles/glfwinfo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include dep/glfw/tests/CMakeFiles/glfwinfo.dir/compiler_depend.make

# Include the progress variables for this target.
include dep/glfw/tests/CMakeFiles/glfwinfo.dir/progress.make

# Include the compile flags for this target's objects.
include dep/glfw/tests/CMakeFiles/glfwinfo.dir/flags.make

dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/flags.make
dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/tests/glfwinfo.c
dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.o"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.o -MF CMakeFiles/glfwinfo.dir/glfwinfo.c.o.d -o CMakeFiles/glfwinfo.dir/glfwinfo.c.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/tests/glfwinfo.c

dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfwinfo.dir/glfwinfo.c.i"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/tests/glfwinfo.c > CMakeFiles/glfwinfo.dir/glfwinfo.c.i

dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfwinfo.dir/glfwinfo.c.s"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/tests/glfwinfo.c -o CMakeFiles/glfwinfo.dir/glfwinfo.c.s

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/flags.make
dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/getopt.c
dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o -MF CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o.d -o CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/getopt.c

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfwinfo.dir/__/deps/getopt.c.i"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/getopt.c > CMakeFiles/glfwinfo.dir/__/deps/getopt.c.i

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfwinfo.dir/__/deps/getopt.c.s"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/getopt.c -o CMakeFiles/glfwinfo.dir/__/deps/getopt.c.s

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/flags.make
dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_gl.c
dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o -MF CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o.d -o CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_gl.c

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.i"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_gl.c > CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.i

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.s"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_gl.c -o CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.s

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/flags.make
dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_vulkan.c
dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o: dep/glfw/tests/CMakeFiles/glfwinfo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o -MF CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o.d -o CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_vulkan.c

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.i"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_vulkan.c > CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.i

dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.s"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/deps/glad_vulkan.c -o CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.s

# Object files for target glfwinfo
glfwinfo_OBJECTS = \
"CMakeFiles/glfwinfo.dir/glfwinfo.c.o" \
"CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o" \
"CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o" \
"CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o"

# External object files for target glfwinfo
glfwinfo_EXTERNAL_OBJECTS =

dep/glfw/tests/glfwinfo: dep/glfw/tests/CMakeFiles/glfwinfo.dir/glfwinfo.c.o
dep/glfw/tests/glfwinfo: dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/getopt.c.o
dep/glfw/tests/glfwinfo: dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_gl.c.o
dep/glfw/tests/glfwinfo: dep/glfw/tests/CMakeFiles/glfwinfo.dir/__/deps/glad_vulkan.c.o
dep/glfw/tests/glfwinfo: dep/glfw/tests/CMakeFiles/glfwinfo.dir/build.make
dep/glfw/tests/glfwinfo: dep/glfw/src/libglfw3.a
dep/glfw/tests/glfwinfo: /usr/lib64/libm.so
dep/glfw/tests/glfwinfo: /usr/lib64/librt.a
dep/glfw/tests/glfwinfo: /usr/lib64/libm.so
dep/glfw/tests/glfwinfo: /usr/lib64/libX11.so
dep/glfw/tests/glfwinfo: dep/glfw/tests/CMakeFiles/glfwinfo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C executable glfwinfo"
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glfwinfo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dep/glfw/tests/CMakeFiles/glfwinfo.dir/build: dep/glfw/tests/glfwinfo
.PHONY : dep/glfw/tests/CMakeFiles/glfwinfo.dir/build

dep/glfw/tests/CMakeFiles/glfwinfo.dir/clean:
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests && $(CMAKE_COMMAND) -P CMakeFiles/glfwinfo.dir/cmake_clean.cmake
.PHONY : dep/glfw/tests/CMakeFiles/glfwinfo.dir/clean

dep/glfw/tests/CMakeFiles/glfwinfo.dir/depend:
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/dep/glfw/tests /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests /home/rs_benatti/Documents/Telecom/IGR/IGR202/TP01-Geometry/build/dep/glfw/tests/CMakeFiles/glfwinfo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dep/glfw/tests/CMakeFiles/glfwinfo.dir/depend

