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
CMAKE_SOURCE_DIR = /opt/opendlv.scaledcars.sources/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/opendlv.scaledcars.build/build.system

# Include any dependencies generated for this target.
include control/example/CMakeFiles/scaledcars-control-example-static.dir/depend.make

# Include the progress variables for this target.
include control/example/CMakeFiles/scaledcars-control-example-static.dir/progress.make

# Include the compile flags for this target's objects.
include control/example/CMakeFiles/scaledcars-control-example-static.dir/flags.make

control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o: control/example/CMakeFiles/scaledcars-control-example-static.dir/flags.make
control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o: /opt/opendlv.scaledcars.sources/code/control/example/src/Example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/opendlv.scaledcars.build/build.system/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o -c /opt/opendlv.scaledcars.sources/code/control/example/src/Example.cpp

control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.i"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/opendlv.scaledcars.sources/code/control/example/src/Example.cpp > CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.i

control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.s"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/opendlv.scaledcars.sources/code/control/example/src/Example.cpp -o CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.s

control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.requires:

.PHONY : control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.requires

control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.provides: control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.requires
	$(MAKE) -f control/example/CMakeFiles/scaledcars-control-example-static.dir/build.make control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.provides.build
.PHONY : control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.provides

control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.provides.build: control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o


# Object files for target scaledcars-control-example-static
scaledcars__control__example__static_OBJECTS = \
"CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o"

# External object files for target scaledcars-control-example-static
scaledcars__control__example__static_EXTERNAL_OBJECTS =

control/example/libscaledcars-control-example-static.a: control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o
control/example/libscaledcars-control-example-static.a: control/example/CMakeFiles/scaledcars-control-example-static.dir/build.make
control/example/libscaledcars-control-example-static.a: control/example/CMakeFiles/scaledcars-control-example-static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/opt/opendlv.scaledcars.build/build.system/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libscaledcars-control-example-static.a"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && $(CMAKE_COMMAND) -P CMakeFiles/scaledcars-control-example-static.dir/cmake_clean_target.cmake
	cd /opt/opendlv.scaledcars.build/build.system/control/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scaledcars-control-example-static.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
control/example/CMakeFiles/scaledcars-control-example-static.dir/build: control/example/libscaledcars-control-example-static.a

.PHONY : control/example/CMakeFiles/scaledcars-control-example-static.dir/build

control/example/CMakeFiles/scaledcars-control-example-static.dir/requires: control/example/CMakeFiles/scaledcars-control-example-static.dir/src/Example.cpp.o.requires

.PHONY : control/example/CMakeFiles/scaledcars-control-example-static.dir/requires

control/example/CMakeFiles/scaledcars-control-example-static.dir/clean:
	cd /opt/opendlv.scaledcars.build/build.system/control/example && $(CMAKE_COMMAND) -P CMakeFiles/scaledcars-control-example-static.dir/cmake_clean.cmake
.PHONY : control/example/CMakeFiles/scaledcars-control-example-static.dir/clean

control/example/CMakeFiles/scaledcars-control-example-static.dir/depend:
	cd /opt/opendlv.scaledcars.build/build.system && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/opendlv.scaledcars.sources/code /opt/opendlv.scaledcars.sources/code/control/example /opt/opendlv.scaledcars.build/build.system /opt/opendlv.scaledcars.build/build.system/control/example /opt/opendlv.scaledcars.build/build.system/control/example/CMakeFiles/scaledcars-control-example-static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : control/example/CMakeFiles/scaledcars-control-example-static.dir/depend
