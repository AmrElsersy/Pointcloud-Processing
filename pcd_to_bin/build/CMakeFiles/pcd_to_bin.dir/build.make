# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/amrelsersy/oustar/pcd_to_bin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amrelsersy/oustar/pcd_to_bin/build

# Include any dependencies generated for this target.
include CMakeFiles/pcd_to_bin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcd_to_bin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcd_to_bin.dir/flags.make

CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.o: CMakeFiles/pcd_to_bin.dir/flags.make
CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.o: ../convert_pcd_to_bin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amrelsersy/oustar/pcd_to_bin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.o -c /home/amrelsersy/oustar/pcd_to_bin/convert_pcd_to_bin.cpp

CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amrelsersy/oustar/pcd_to_bin/convert_pcd_to_bin.cpp > CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.i

CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amrelsersy/oustar/pcd_to_bin/convert_pcd_to_bin.cpp -o CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.s

# Object files for target pcd_to_bin
pcd_to_bin_OBJECTS = \
"CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.o"

# External object files for target pcd_to_bin
pcd_to_bin_EXTERNAL_OBJECTS =

pcd_to_bin: CMakeFiles/pcd_to_bin.dir/convert_pcd_to_bin.cpp.o
pcd_to_bin: CMakeFiles/pcd_to_bin.dir/build.make
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_people.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libboost_system.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libboost_regex.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libqhull.so
pcd_to_bin: /usr/lib/libOpenNI.so
pcd_to_bin: /usr/lib/libOpenNI2.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libfreetype.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libz.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libjpeg.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpng.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libtiff.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libexpat.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_features.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_search.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_io.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libpcl_common.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libfreetype.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libz.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libGLEW.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libSM.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libICE.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libX11.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libXext.so
pcd_to_bin: /usr/lib/x86_64-linux-gnu/libXt.so
pcd_to_bin: CMakeFiles/pcd_to_bin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amrelsersy/oustar/pcd_to_bin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pcd_to_bin"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcd_to_bin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcd_to_bin.dir/build: pcd_to_bin

.PHONY : CMakeFiles/pcd_to_bin.dir/build

CMakeFiles/pcd_to_bin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcd_to_bin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcd_to_bin.dir/clean

CMakeFiles/pcd_to_bin.dir/depend:
	cd /home/amrelsersy/oustar/pcd_to_bin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amrelsersy/oustar/pcd_to_bin /home/amrelsersy/oustar/pcd_to_bin /home/amrelsersy/oustar/pcd_to_bin/build /home/amrelsersy/oustar/pcd_to_bin/build /home/amrelsersy/oustar/pcd_to_bin/build/CMakeFiles/pcd_to_bin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcd_to_bin.dir/depend

