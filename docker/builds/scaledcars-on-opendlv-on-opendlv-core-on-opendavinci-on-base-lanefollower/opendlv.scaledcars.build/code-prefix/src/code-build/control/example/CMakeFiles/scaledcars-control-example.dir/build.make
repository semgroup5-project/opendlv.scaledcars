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
CMAKE_BINARY_DIR = /opt/opendlv.scaledcars.build/code-prefix/src/code-build

# Include any dependencies generated for this target.
include control/example/CMakeFiles/scaledcars-control-example.dir/depend.make

# Include the progress variables for this target.
include control/example/CMakeFiles/scaledcars-control-example.dir/progress.make

# Include the compile flags for this target's objects.
include control/example/CMakeFiles/scaledcars-control-example.dir/flags.make

control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o: control/example/CMakeFiles/scaledcars-control-example.dir/flags.make
control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o: /opt/opendlv.scaledcars.sources/code/control/example/apps/scaledcars-control-example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/opendlv.scaledcars.build/code-prefix/src/code-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o -c /opt/opendlv.scaledcars.sources/code/control/example/apps/scaledcars-control-example.cpp

control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.i"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/opendlv.scaledcars.sources/code/control/example/apps/scaledcars-control-example.cpp > CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.i

control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.s"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/opendlv.scaledcars.sources/code/control/example/apps/scaledcars-control-example.cpp -o CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.s

control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.requires:

.PHONY : control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.requires

control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.provides: control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.requires
	$(MAKE) -f control/example/CMakeFiles/scaledcars-control-example.dir/build.make control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.provides.build
.PHONY : control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.provides

control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.provides.build: control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o


# Object files for target scaledcars-control-example
scaledcars__control__example_OBJECTS = \
"CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o"

# External object files for target scaledcars-control-example
scaledcars__control__example_EXTERNAL_OBJECTS =

control/example/scaledcars-control-example: control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o
control/example/scaledcars-control-example: control/example/CMakeFiles/scaledcars-control-example.dir/build.make
control/example/scaledcars-control-example: control/example/libscaledcars-control-example-static.a
control/example/scaledcars-control-example: /opt/od4/lib/libopendavinci.so
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/librt.a
control/example/scaledcars-control-example: /opt/od4/lib/libautomotivedata.so
control/example/scaledcars-control-example: /opt/opendlv.scaledcars/lib/libodvdscaledcarsdatamodel.so
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
control/example/scaledcars-control-example: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
control/example/scaledcars-control-example: control/example/CMakeFiles/scaledcars-control-example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/opt/opendlv.scaledcars.build/code-prefix/src/code-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable scaledcars-control-example"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scaledcars-control-example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
control/example/CMakeFiles/scaledcars-control-example.dir/build: control/example/scaledcars-control-example

.PHONY : control/example/CMakeFiles/scaledcars-control-example.dir/build

control/example/CMakeFiles/scaledcars-control-example.dir/requires: control/example/CMakeFiles/scaledcars-control-example.dir/apps/scaledcars-control-example.cpp.o.requires

.PHONY : control/example/CMakeFiles/scaledcars-control-example.dir/requires

control/example/CMakeFiles/scaledcars-control-example.dir/clean:
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/example && $(CMAKE_COMMAND) -P CMakeFiles/scaledcars-control-example.dir/cmake_clean.cmake
.PHONY : control/example/CMakeFiles/scaledcars-control-example.dir/clean

control/example/CMakeFiles/scaledcars-control-example.dir/depend:
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/opendlv.scaledcars.sources/code /opt/opendlv.scaledcars.sources/code/control/example /opt/opendlv.scaledcars.build/code-prefix/src/code-build /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/example /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/example/CMakeFiles/scaledcars-control-example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : control/example/CMakeFiles/scaledcars-control-example.dir/depend

