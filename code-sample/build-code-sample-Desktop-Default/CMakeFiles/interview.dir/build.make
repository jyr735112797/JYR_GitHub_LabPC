# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/jyr/Desktop/code-sample/code-sample

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default

# Include any dependencies generated for this target.
include CMakeFiles/interview.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/interview.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/interview.dir/flags.make

CMakeFiles/interview.dir/src/geometry.cpp.o: CMakeFiles/interview.dir/flags.make
CMakeFiles/interview.dir/src/geometry.cpp.o: ../src/geometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/interview.dir/src/geometry.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interview.dir/src/geometry.cpp.o -c /home/jyr/Desktop/code-sample/code-sample/src/geometry.cpp

CMakeFiles/interview.dir/src/geometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interview.dir/src/geometry.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jyr/Desktop/code-sample/code-sample/src/geometry.cpp > CMakeFiles/interview.dir/src/geometry.cpp.i

CMakeFiles/interview.dir/src/geometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interview.dir/src/geometry.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jyr/Desktop/code-sample/code-sample/src/geometry.cpp -o CMakeFiles/interview.dir/src/geometry.cpp.s

CMakeFiles/interview.dir/src/geometry.cpp.o.requires:

.PHONY : CMakeFiles/interview.dir/src/geometry.cpp.o.requires

CMakeFiles/interview.dir/src/geometry.cpp.o.provides: CMakeFiles/interview.dir/src/geometry.cpp.o.requires
	$(MAKE) -f CMakeFiles/interview.dir/build.make CMakeFiles/interview.dir/src/geometry.cpp.o.provides.build
.PHONY : CMakeFiles/interview.dir/src/geometry.cpp.o.provides

CMakeFiles/interview.dir/src/geometry.cpp.o.provides.build: CMakeFiles/interview.dir/src/geometry.cpp.o


CMakeFiles/interview.dir/src/utils.cpp.o: CMakeFiles/interview.dir/flags.make
CMakeFiles/interview.dir/src/utils.cpp.o: ../src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/interview.dir/src/utils.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interview.dir/src/utils.cpp.o -c /home/jyr/Desktop/code-sample/code-sample/src/utils.cpp

CMakeFiles/interview.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interview.dir/src/utils.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jyr/Desktop/code-sample/code-sample/src/utils.cpp > CMakeFiles/interview.dir/src/utils.cpp.i

CMakeFiles/interview.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interview.dir/src/utils.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jyr/Desktop/code-sample/code-sample/src/utils.cpp -o CMakeFiles/interview.dir/src/utils.cpp.s

CMakeFiles/interview.dir/src/utils.cpp.o.requires:

.PHONY : CMakeFiles/interview.dir/src/utils.cpp.o.requires

CMakeFiles/interview.dir/src/utils.cpp.o.provides: CMakeFiles/interview.dir/src/utils.cpp.o.requires
	$(MAKE) -f CMakeFiles/interview.dir/build.make CMakeFiles/interview.dir/src/utils.cpp.o.provides.build
.PHONY : CMakeFiles/interview.dir/src/utils.cpp.o.provides

CMakeFiles/interview.dir/src/utils.cpp.o.provides.build: CMakeFiles/interview.dir/src/utils.cpp.o


CMakeFiles/interview.dir/src/svg.cpp.o: CMakeFiles/interview.dir/flags.make
CMakeFiles/interview.dir/src/svg.cpp.o: ../src/svg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/interview.dir/src/svg.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interview.dir/src/svg.cpp.o -c /home/jyr/Desktop/code-sample/code-sample/src/svg.cpp

CMakeFiles/interview.dir/src/svg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interview.dir/src/svg.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jyr/Desktop/code-sample/code-sample/src/svg.cpp > CMakeFiles/interview.dir/src/svg.cpp.i

CMakeFiles/interview.dir/src/svg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interview.dir/src/svg.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jyr/Desktop/code-sample/code-sample/src/svg.cpp -o CMakeFiles/interview.dir/src/svg.cpp.s

CMakeFiles/interview.dir/src/svg.cpp.o.requires:

.PHONY : CMakeFiles/interview.dir/src/svg.cpp.o.requires

CMakeFiles/interview.dir/src/svg.cpp.o.provides: CMakeFiles/interview.dir/src/svg.cpp.o.requires
	$(MAKE) -f CMakeFiles/interview.dir/build.make CMakeFiles/interview.dir/src/svg.cpp.o.provides.build
.PHONY : CMakeFiles/interview.dir/src/svg.cpp.o.provides

CMakeFiles/interview.dir/src/svg.cpp.o.provides.build: CMakeFiles/interview.dir/src/svg.cpp.o


# Object files for target interview
interview_OBJECTS = \
"CMakeFiles/interview.dir/src/geometry.cpp.o" \
"CMakeFiles/interview.dir/src/utils.cpp.o" \
"CMakeFiles/interview.dir/src/svg.cpp.o"

# External object files for target interview
interview_EXTERNAL_OBJECTS =

lib/libinterview.a: CMakeFiles/interview.dir/src/geometry.cpp.o
lib/libinterview.a: CMakeFiles/interview.dir/src/utils.cpp.o
lib/libinterview.a: CMakeFiles/interview.dir/src/svg.cpp.o
lib/libinterview.a: CMakeFiles/interview.dir/build.make
lib/libinterview.a: CMakeFiles/interview.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library lib/libinterview.a"
	$(CMAKE_COMMAND) -P CMakeFiles/interview.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/interview.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/interview.dir/build: lib/libinterview.a

.PHONY : CMakeFiles/interview.dir/build

CMakeFiles/interview.dir/requires: CMakeFiles/interview.dir/src/geometry.cpp.o.requires
CMakeFiles/interview.dir/requires: CMakeFiles/interview.dir/src/utils.cpp.o.requires
CMakeFiles/interview.dir/requires: CMakeFiles/interview.dir/src/svg.cpp.o.requires

.PHONY : CMakeFiles/interview.dir/requires

CMakeFiles/interview.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/interview.dir/cmake_clean.cmake
.PHONY : CMakeFiles/interview.dir/clean

CMakeFiles/interview.dir/depend:
	cd /home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jyr/Desktop/code-sample/code-sample /home/jyr/Desktop/code-sample/code-sample /home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default /home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default /home/jyr/Desktop/code-sample/code-sample/build-code-sample-Desktop-Default/CMakeFiles/interview.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/interview.dir/depend

