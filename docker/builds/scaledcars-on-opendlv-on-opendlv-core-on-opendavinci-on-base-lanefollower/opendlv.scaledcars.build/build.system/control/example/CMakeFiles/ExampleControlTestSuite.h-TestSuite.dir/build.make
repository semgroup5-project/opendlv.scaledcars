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
include control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/depend.make

# Include the progress variables for this target.
include control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/progress.make

# Include the compile flags for this target's objects.
include control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/flags.make

control/example/ExampleControlTestSuite.h-TestSuite.cpp: /opt/opendlv.scaledcars.sources/code/control/example/testsuites/ExampleControlTestSuite.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/build.system/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ExampleControlTestSuite.h-TestSuite.cpp"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && /usr/bin/python2.7 /opt/opendlv.scaledcars.sources/thirdparty/cxxtest/cxxtestgen --xunit-printer --have-eh --world=scaledcars-control-example-ExampleControlTestSuite.h -o /opt/opendlv.scaledcars.build/build.system/control/example/ExampleControlTestSuite.h-TestSuite.cpp /opt/opendlv.scaledcars.sources/code/control/example/testsuites/ExampleControlTestSuite.h

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o: control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/flags.make
control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o: control/example/ExampleControlTestSuite.h-TestSuite.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/opendlv.scaledcars.build/build.system/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -Wno-effc++ -Wno-float-equal -Wno-error=suggest-attribute=noreturn -o CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o -c /opt/opendlv.scaledcars.build/build.system/control/example/ExampleControlTestSuite.h-TestSuite.cpp

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.i"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -Wno-effc++ -Wno-float-equal -Wno-error=suggest-attribute=noreturn -E /opt/opendlv.scaledcars.build/build.system/control/example/ExampleControlTestSuite.h-TestSuite.cpp > CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.i

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.s"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -Wno-effc++ -Wno-float-equal -Wno-error=suggest-attribute=noreturn -S /opt/opendlv.scaledcars.build/build.system/control/example/ExampleControlTestSuite.h-TestSuite.cpp -o CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.s

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.requires:

.PHONY : control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.requires

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.provides: control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.requires
	$(MAKE) -f control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/build.make control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.provides.build
.PHONY : control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.provides

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.provides.build: control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o


# Object files for target ExampleControlTestSuite.h-TestSuite
ExampleControlTestSuite_h__TestSuite_OBJECTS = \
"CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o"

# External object files for target ExampleControlTestSuite.h-TestSuite
ExampleControlTestSuite_h__TestSuite_EXTERNAL_OBJECTS =

control/example/ExampleControlTestSuite.h-TestSuite: control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o
control/example/ExampleControlTestSuite.h-TestSuite: control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/build.make
control/example/ExampleControlTestSuite.h-TestSuite: control/example/libscaledcars-control-example-static.a
control/example/ExampleControlTestSuite.h-TestSuite: /opt/od4/lib/libopendavinci.so
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/librt.a
control/example/ExampleControlTestSuite.h-TestSuite: /opt/od4/lib/libautomotivedata.so
control/example/ExampleControlTestSuite.h-TestSuite: /opt/opendlv.scaledcars/lib/libodvdscaledcarsdatamodel.so
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
control/example/ExampleControlTestSuite.h-TestSuite: control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/opt/opendlv.scaledcars.build/build.system/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ExampleControlTestSuite.h-TestSuite"
	cd /opt/opendlv.scaledcars.build/build.system/control/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/build: control/example/ExampleControlTestSuite.h-TestSuite

.PHONY : control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/build

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/requires: control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/ExampleControlTestSuite.h-TestSuite.cpp.o.requires

.PHONY : control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/requires

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/clean:
	cd /opt/opendlv.scaledcars.build/build.system/control/example && $(CMAKE_COMMAND) -P CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/cmake_clean.cmake
.PHONY : control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/clean

control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/depend: control/example/ExampleControlTestSuite.h-TestSuite.cpp
	cd /opt/opendlv.scaledcars.build/build.system && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/opendlv.scaledcars.sources/code /opt/opendlv.scaledcars.sources/code/control/example /opt/opendlv.scaledcars.build/build.system /opt/opendlv.scaledcars.build/build.system/control/example /opt/opendlv.scaledcars.build/build.system/control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : control/example/CMakeFiles/ExampleControlTestSuite.h-TestSuite.dir/depend

