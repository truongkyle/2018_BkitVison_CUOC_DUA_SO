# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build

# Include any dependencies generated for this target.
include team207/CMakeFiles/team207_node.dir/depend.make

# Include the progress variables for this target.
include team207/CMakeFiles/team207_node.dir/progress.make

# Include the compile flags for this target's objects.
include team207/CMakeFiles/team207_node.dir/flags.make

team207/CMakeFiles/team207_node.dir/src/main.cpp.o: team207/CMakeFiles/team207_node.dir/flags.make
team207/CMakeFiles/team207_node.dir/src/main.cpp.o: /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/src/team207/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object team207/CMakeFiles/team207_node.dir/src/main.cpp.o"
	cd /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/team207 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/team207_node.dir/src/main.cpp.o -c /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/src/team207/src/main.cpp

team207/CMakeFiles/team207_node.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/team207_node.dir/src/main.cpp.i"
	cd /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/team207 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/src/team207/src/main.cpp > CMakeFiles/team207_node.dir/src/main.cpp.i

team207/CMakeFiles/team207_node.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/team207_node.dir/src/main.cpp.s"
	cd /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/team207 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/src/team207/src/main.cpp -o CMakeFiles/team207_node.dir/src/main.cpp.s

team207/CMakeFiles/team207_node.dir/src/main.cpp.o.requires:

.PHONY : team207/CMakeFiles/team207_node.dir/src/main.cpp.o.requires

team207/CMakeFiles/team207_node.dir/src/main.cpp.o.provides: team207/CMakeFiles/team207_node.dir/src/main.cpp.o.requires
	$(MAKE) -f team207/CMakeFiles/team207_node.dir/build.make team207/CMakeFiles/team207_node.dir/src/main.cpp.o.provides.build
.PHONY : team207/CMakeFiles/team207_node.dir/src/main.cpp.o.provides

team207/CMakeFiles/team207_node.dir/src/main.cpp.o.provides.build: team207/CMakeFiles/team207_node.dir/src/main.cpp.o


# Object files for target team207_node
team207_node_OBJECTS = \
"CMakeFiles/team207_node.dir/src/main.cpp.o"

# External object files for target team207_node
team207_node_EXTERNAL_OBJECTS =

/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: team207/CMakeFiles/team207_node.dir/src/main.cpp.o
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: team207/CMakeFiles/team207_node.dir/build.make
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/libteam207.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libcv_bridge.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libimage_transport.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libmessage_filters.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libclass_loader.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/libPocoFoundation.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libroslib.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/librospack.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libroscpp.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/librosconsole.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/librostime.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /opt/ros/melodic/lib/libcpp_common.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node: team207/CMakeFiles/team207_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node"
	cd /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/team207 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/team207_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
team207/CMakeFiles/team207_node.dir/build: /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/devel/lib/team207/team207_node

.PHONY : team207/CMakeFiles/team207_node.dir/build

team207/CMakeFiles/team207_node.dir/requires: team207/CMakeFiles/team207_node.dir/src/main.cpp.o.requires

.PHONY : team207/CMakeFiles/team207_node.dir/requires

team207/CMakeFiles/team207_node.dir/clean:
	cd /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/team207 && $(CMAKE_COMMAND) -P CMakeFiles/team207_node.dir/cmake_clean.cmake
.PHONY : team207/CMakeFiles/team207_node.dir/clean

team207/CMakeFiles/team207_node.dir/depend:
	cd /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/src /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/src/team207 /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/team207 /home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/build/team207/CMakeFiles/team207_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : team207/CMakeFiles/team207_node.dir/depend

