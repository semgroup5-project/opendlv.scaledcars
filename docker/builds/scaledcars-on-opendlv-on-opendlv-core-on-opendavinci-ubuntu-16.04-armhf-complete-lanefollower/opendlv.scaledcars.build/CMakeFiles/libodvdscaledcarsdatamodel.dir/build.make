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
CMAKE_SOURCE_DIR = /opt/opendlv.scaledcars.sources

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/opendlv.scaledcars.build

# Utility rule file for libodvdscaledcarsdatamodel.

# Include the progress variables for this target.
include CMakeFiles/libodvdscaledcarsdatamodel.dir/progress.make

CMakeFiles/libodvdscaledcarsdatamodel: CMakeFiles/libodvdscaledcarsdatamodel-complete


CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-install
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-mkdir
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-download
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-update
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-patch
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-build
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-install
CMakeFiles/libodvdscaledcarsdatamodel-complete: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'libodvdscaledcarsdatamodel'"
	/usr/bin/cmake -E make_directory /opt/opendlv.scaledcars.build/CMakeFiles
	/usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/CMakeFiles/libodvdscaledcarsdatamodel-complete
	/usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-done

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-install: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-build
libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-install: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'libodvdscaledcarsdatamodel'"
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/cmake --build /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build --target install
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/cmake --build /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build --target help
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-install

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'libodvdscaledcarsdatamodel'"
	/usr/bin/cmake -E make_directory /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel
	/usr/bin/cmake -E make_directory /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build
	/usr/bin/cmake -E make_directory /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix
	/usr/bin/cmake -E make_directory /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/tmp
	/usr/bin/cmake -E make_directory /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp
	/usr/bin/cmake -E make_directory /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src
	/usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-mkdir

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-download: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'libodvdscaledcarsdatamodel'"
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src && /usr/bin/cmake -E echo_append
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src && /usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-download

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-update: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'libodvdscaledcarsdatamodel'"
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel && cd /opt/opendlv.scaledcars.build && /usr/bin/java -jar /opt/od4/bin/odDataStructureGenerator-latest.jar --withCMake /opt/opendlv.scaledcars.sources/resources/odvd/ODVDScaledCarsDataModel.odvd
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel && /usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-update

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-patch: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'libodvdscaledcarsdatamodel'"
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel && /usr/bin/cmake -E echo_append
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel && /usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-patch

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure: libodvdscaledcarsdatamodel-cleanup-prefix/src/libodvdscaledcarsdatamodel-cleanup-stamp/libodvdscaledcarsdatamodel-cleanup-done
libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure: libodvdscaledcarsdatamodel-prefix/tmp/libodvdscaledcarsdatamodel-cfgcmd.txt
libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-update
libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'libodvdscaledcarsdatamodel'"
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/opt/opendlv.scaledcars -DOPENDAVINCI_DIR=/opt/od4 -DCMAKE_TOOLCHAIN_FILE= -DCXXTEST_INCLUDE_DIR=/opt/opendlv.scaledcars.sources/thirdparty/cxxtest "-GUnix Makefiles" /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-build: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'libodvdscaledcarsdatamodel'"
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && $(MAKE)
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-build

libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-test: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/opt/opendlv.scaledcars.build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Performing test step for 'libodvdscaledcarsdatamodel'"
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/ctest
	cd /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-build && /usr/bin/cmake -E touch /opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-test

libodvdscaledcarsdatamodel: CMakeFiles/libodvdscaledcarsdatamodel
libodvdscaledcarsdatamodel: CMakeFiles/libodvdscaledcarsdatamodel-complete
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-install
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-mkdir
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-download
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-update
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-patch
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-configure
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-build
libodvdscaledcarsdatamodel: libodvdscaledcarsdatamodel-prefix/src/libodvdscaledcarsdatamodel-stamp/libodvdscaledcarsdatamodel-test
libodvdscaledcarsdatamodel: CMakeFiles/libodvdscaledcarsdatamodel.dir/build.make

.PHONY : libodvdscaledcarsdatamodel

# Rule to build all files generated by this target.
CMakeFiles/libodvdscaledcarsdatamodel.dir/build: libodvdscaledcarsdatamodel

.PHONY : CMakeFiles/libodvdscaledcarsdatamodel.dir/build

CMakeFiles/libodvdscaledcarsdatamodel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/libodvdscaledcarsdatamodel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/libodvdscaledcarsdatamodel.dir/clean

CMakeFiles/libodvdscaledcarsdatamodel.dir/depend:
	cd /opt/opendlv.scaledcars.build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/opendlv.scaledcars.sources /opt/opendlv.scaledcars.sources /opt/opendlv.scaledcars.build /opt/opendlv.scaledcars.build /opt/opendlv.scaledcars.build/CMakeFiles/libodvdscaledcarsdatamodel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/libodvdscaledcarsdatamodel.dir/depend

