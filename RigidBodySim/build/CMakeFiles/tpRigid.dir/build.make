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
CMAKE_SOURCE_DIR = /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build

# Include any dependencies generated for this target.
include CMakeFiles/tpRigid.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/tpRigid.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/tpRigid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tpRigid.dir/flags.make

CMakeFiles/tpRigid.dir/src/main.cpp.o: CMakeFiles/tpRigid.dir/flags.make
CMakeFiles/tpRigid.dir/src/main.cpp.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/main.cpp
CMakeFiles/tpRigid.dir/src/main.cpp.o: CMakeFiles/tpRigid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tpRigid.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tpRigid.dir/src/main.cpp.o -MF CMakeFiles/tpRigid.dir/src/main.cpp.o.d -o CMakeFiles/tpRigid.dir/src/main.cpp.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/main.cpp

CMakeFiles/tpRigid.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tpRigid.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/main.cpp > CMakeFiles/tpRigid.dir/src/main.cpp.i

CMakeFiles/tpRigid.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tpRigid.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/main.cpp -o CMakeFiles/tpRigid.dir/src/main.cpp.s

CMakeFiles/tpRigid.dir/src/Mesh.cpp.o: CMakeFiles/tpRigid.dir/flags.make
CMakeFiles/tpRigid.dir/src/Mesh.cpp.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/Mesh.cpp
CMakeFiles/tpRigid.dir/src/Mesh.cpp.o: CMakeFiles/tpRigid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/tpRigid.dir/src/Mesh.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tpRigid.dir/src/Mesh.cpp.o -MF CMakeFiles/tpRigid.dir/src/Mesh.cpp.o.d -o CMakeFiles/tpRigid.dir/src/Mesh.cpp.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/Mesh.cpp

CMakeFiles/tpRigid.dir/src/Mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tpRigid.dir/src/Mesh.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/Mesh.cpp > CMakeFiles/tpRigid.dir/src/Mesh.cpp.i

CMakeFiles/tpRigid.dir/src/Mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tpRigid.dir/src/Mesh.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/Mesh.cpp -o CMakeFiles/tpRigid.dir/src/Mesh.cpp.s

CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o: CMakeFiles/tpRigid.dir/flags.make
CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/ShaderProgram.cpp
CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o: CMakeFiles/tpRigid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o -MF CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o.d -o CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/ShaderProgram.cpp

CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/ShaderProgram.cpp > CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.i

CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/src/ShaderProgram.cpp -o CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.s

CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o: CMakeFiles/tpRigid.dir/flags.make
CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o: /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/dep/glad/src/glad.c
CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o: CMakeFiles/tpRigid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o -MF CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o.d -o CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o -c /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/dep/glad/src/glad.c

CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/dep/glad/src/glad.c > CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.i

CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/dep/glad/src/glad.c -o CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.s

# Object files for target tpRigid
tpRigid_OBJECTS = \
"CMakeFiles/tpRigid.dir/src/main.cpp.o" \
"CMakeFiles/tpRigid.dir/src/Mesh.cpp.o" \
"CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o" \
"CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o"

# External object files for target tpRigid
tpRigid_EXTERNAL_OBJECTS =

tpRigid: CMakeFiles/tpRigid.dir/src/main.cpp.o
tpRigid: CMakeFiles/tpRigid.dir/src/Mesh.cpp.o
tpRigid: CMakeFiles/tpRigid.dir/src/ShaderProgram.cpp.o
tpRigid: CMakeFiles/tpRigid.dir/dep/glad/src/glad.c.o
tpRigid: CMakeFiles/tpRigid.dir/build.make
tpRigid: dep/glfw/src/libglfw3.a
tpRigid: /usr/lib64/librt.a
tpRigid: /usr/lib64/libm.so
tpRigid: /usr/lib64/libX11.so
tpRigid: CMakeFiles/tpRigid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable tpRigid"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tpRigid.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/cmake -E copy /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build/tpRigid /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim

# Rule to build all files generated by this target.
CMakeFiles/tpRigid.dir/build: tpRigid
.PHONY : CMakeFiles/tpRigid.dir/build

CMakeFiles/tpRigid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tpRigid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tpRigid.dir/clean

CMakeFiles/tpRigid.dir/depend:
	cd /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build /home/rs_benatti/Documents/Telecom/IGR/IGR202/RigidSim/build/CMakeFiles/tpRigid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tpRigid.dir/depend

