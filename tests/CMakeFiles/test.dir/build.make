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
CMAKE_SOURCE_DIR = /home/araujo/Documents/cbrp-lagrangean/tests

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/araujo/Documents/cbrp-lagrangean/tests

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/test_common.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/test_common.cpp.o: test_common.cpp
CMakeFiles/test.dir/test_common.cpp.o: CMakeFiles/test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/araujo/Documents/cbrp-lagrangean/tests/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/test_common.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test.dir/test_common.cpp.o -MF CMakeFiles/test.dir/test_common.cpp.o.d -o CMakeFiles/test.dir/test_common.cpp.o -c /home/araujo/Documents/cbrp-lagrangean/tests/test_common.cpp

CMakeFiles/test.dir/test_common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/test_common.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/araujo/Documents/cbrp-lagrangean/tests/test_common.cpp > CMakeFiles/test.dir/test_common.cpp.i

CMakeFiles/test.dir/test_common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/test_common.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/araujo/Documents/cbrp-lagrangean/tests/test_common.cpp -o CMakeFiles/test.dir/test_common.cpp.s

CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o: /home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp
CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o: CMakeFiles/test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/araujo/Documents/cbrp-lagrangean/tests/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o -MF CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o.d -o CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o -c /home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp

CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp > CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.i

CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp -o CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.s

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/test_common.cpp.o" \
"CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/test_common.cpp.o
test: CMakeFiles/test.dir/home/araujo/Documents/cbrp-lagrangean/src/classes/Graph.cpp.o
test: CMakeFiles/test.dir/build.make
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/araujo/Documents/cbrp-lagrangean/tests/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test
.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /home/araujo/Documents/cbrp-lagrangean/tests && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/araujo/Documents/cbrp-lagrangean/tests /home/araujo/Documents/cbrp-lagrangean/tests /home/araujo/Documents/cbrp-lagrangean/tests /home/araujo/Documents/cbrp-lagrangean/tests /home/araujo/Documents/cbrp-lagrangean/tests/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend
