# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_SOURCE_DIR = /home/rs_benatti/Téléchargements/glfw-3.3.8

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rs_benatti/Téléchargements/glfw-3.3.8/build

# Include any dependencies generated for this target.
include tests/CMakeFiles/opacity.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tests/CMakeFiles/opacity.dir/compiler_depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/opacity.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/opacity.dir/flags.make

tests/CMakeFiles/opacity.dir/opacity.c.o: tests/CMakeFiles/opacity.dir/flags.make
tests/CMakeFiles/opacity.dir/opacity.c.o: /home/rs_benatti/Téléchargements/glfw-3.3.8/tests/opacity.c
tests/CMakeFiles/opacity.dir/opacity.c.o: tests/CMakeFiles/opacity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Téléchargements/glfw-3.3.8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object tests/CMakeFiles/opacity.dir/opacity.c.o"
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT tests/CMakeFiles/opacity.dir/opacity.c.o -MF CMakeFiles/opacity.dir/opacity.c.o.d -o CMakeFiles/opacity.dir/opacity.c.o -c /home/rs_benatti/Téléchargements/glfw-3.3.8/tests/opacity.c

tests/CMakeFiles/opacity.dir/opacity.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/opacity.dir/opacity.c.i"
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rs_benatti/Téléchargements/glfw-3.3.8/tests/opacity.c > CMakeFiles/opacity.dir/opacity.c.i

tests/CMakeFiles/opacity.dir/opacity.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/opacity.dir/opacity.c.s"
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rs_benatti/Téléchargements/glfw-3.3.8/tests/opacity.c -o CMakeFiles/opacity.dir/opacity.c.s

tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.o: tests/CMakeFiles/opacity.dir/flags.make
tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.o: /home/rs_benatti/Téléchargements/glfw-3.3.8/deps/glad_gl.c
tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.o: tests/CMakeFiles/opacity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Téléchargements/glfw-3.3.8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.o"
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.o -MF CMakeFiles/opacity.dir/__/deps/glad_gl.c.o.d -o CMakeFiles/opacity.dir/__/deps/glad_gl.c.o -c /home/rs_benatti/Téléchargements/glfw-3.3.8/deps/glad_gl.c

tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/opacity.dir/__/deps/glad_gl.c.i"
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rs_benatti/Téléchargements/glfw-3.3.8/deps/glad_gl.c > CMakeFiles/opacity.dir/__/deps/glad_gl.c.i

tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/opacity.dir/__/deps/glad_gl.c.s"
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rs_benatti/Téléchargements/glfw-3.3.8/deps/glad_gl.c -o CMakeFiles/opacity.dir/__/deps/glad_gl.c.s

# Object files for target opacity
opacity_OBJECTS = \
"CMakeFiles/opacity.dir/opacity.c.o" \
"CMakeFiles/opacity.dir/__/deps/glad_gl.c.o"

# External object files for target opacity
opacity_EXTERNAL_OBJECTS =

tests/opacity: tests/CMakeFiles/opacity.dir/opacity.c.o
tests/opacity: tests/CMakeFiles/opacity.dir/__/deps/glad_gl.c.o
tests/opacity: tests/CMakeFiles/opacity.dir/build.make
tests/opacity: src/libglfw3.a
tests/opacity: /usr/lib64/libm.so
tests/opacity: /usr/lib64/librt.a
tests/opacity: /usr/lib64/libm.so
tests/opacity: /usr/lib64/libX11.so
tests/opacity: tests/CMakeFiles/opacity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rs_benatti/Téléchargements/glfw-3.3.8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable opacity"
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opacity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/opacity.dir/build: tests/opacity
.PHONY : tests/CMakeFiles/opacity.dir/build

tests/CMakeFiles/opacity.dir/clean:
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests && $(CMAKE_COMMAND) -P CMakeFiles/opacity.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/opacity.dir/clean

tests/CMakeFiles/opacity.dir/depend:
	cd /home/rs_benatti/Téléchargements/glfw-3.3.8/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rs_benatti/Téléchargements/glfw-3.3.8 /home/rs_benatti/Téléchargements/glfw-3.3.8/tests /home/rs_benatti/Téléchargements/glfw-3.3.8/build /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests /home/rs_benatti/Téléchargements/glfw-3.3.8/build/tests/CMakeFiles/opacity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/opacity.dir/depend

