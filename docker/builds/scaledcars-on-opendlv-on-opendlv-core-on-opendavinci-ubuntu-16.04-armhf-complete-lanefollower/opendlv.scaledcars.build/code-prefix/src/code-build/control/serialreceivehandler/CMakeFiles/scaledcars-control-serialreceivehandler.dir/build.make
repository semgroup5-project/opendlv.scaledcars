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
include control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/depend.make

# Include the progress variables for this target.
include control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/progress.make

# Include the compile flags for this target's objects.
include control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/flags.make

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o: control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/flags.make
control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o: /opt/opendlv.scaledcars.sources/code/control/serialreceivehandler/apps/scaledcars-control-serialreceivehandler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/opendlv.scaledcars.build/code-prefix/src/code-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/serialreceivehandler && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o -c /opt/opendlv.scaledcars.sources/code/control/serialreceivehandler/apps/scaledcars-control-serialreceivehandler.cpp

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.i"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/serialreceivehandler && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/opendlv.scaledcars.sources/code/control/serialreceivehandler/apps/scaledcars-control-serialreceivehandler.cpp > CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.i

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.s"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/serialreceivehandler && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/opendlv.scaledcars.sources/code/control/serialreceivehandler/apps/scaledcars-control-serialreceivehandler.cpp -o CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.s

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.requires:

.PHONY : control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.requires

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.provides: control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.requires
	$(MAKE) -f control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/build.make control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.provides.build
.PHONY : control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.provides

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.provides.build: control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o


# Object files for target scaledcars-control-serialreceivehandler
scaledcars__control__serialreceivehandler_OBJECTS = \
"CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o"

# External object files for target scaledcars-control-serialreceivehandler
scaledcars__control__serialreceivehandler_EXTERNAL_OBJECTS =

control/serialreceivehandler/scaledcars-control-serialreceivehandler: control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o
control/serialreceivehandler/scaledcars-control-serialreceivehandler: control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/build.make
control/serialreceivehandler/scaledcars-control-serialreceivehandler: control/serialreceivehandler/libscaledcars-control-serialreceivehandler-static.a
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /opt/od4/lib/libopendavinci.so
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/librt.a
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /opt/od4/lib/libautomotivedata.so
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /opt/opendlv.scaledcars/lib/libodvdscaledcarsdatamodel.so
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
control/serialreceivehandler/scaledcars-control-serialreceivehandler: control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/opt/opendlv.scaledcars.build/code-prefix/src/code-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable scaledcars-control-serialreceivehandler"
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/serialreceivehandler && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scaledcars-control-serialreceivehandler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/build: control/serialreceivehandler/scaledcars-control-serialreceivehandler

.PHONY : control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/build

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/requires: control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/apps/scaledcars-control-serialreceivehandler.cpp.o.requires

.PHONY : control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/requires

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/clean:
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/serialreceivehandler && $(CMAKE_COMMAND) -P CMakeFiles/scaledcars-control-serialreceivehandler.dir/cmake_clean.cmake
.PHONY : control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/clean

control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/depend:
	cd /opt/opendlv.scaledcars.build/code-prefix/src/code-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/opendlv.scaledcars.sources/code /opt/opendlv.scaledcars.sources/code/control/serialreceivehandler /opt/opendlv.scaledcars.build/code-prefix/src/code-build /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/serialreceivehandler /opt/opendlv.scaledcars.build/code-prefix/src/code-build/control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : control/serialreceivehandler/CMakeFiles/scaledcars-control-serialreceivehandler.dir/depend

