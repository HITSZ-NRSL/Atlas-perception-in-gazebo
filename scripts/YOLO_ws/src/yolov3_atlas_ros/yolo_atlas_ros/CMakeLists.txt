# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(yolo_atlas_ros)


# Compile options
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

set(INC_PATH $ENV{DDK_PATH})

if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif ()

set(LIB_PATH $ENV{NPU_HOST_LIB})

if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/acllib/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif ()

#if (${CMAKE_CXX_COMPILER} MATCHES "aarch")
set(ARCH "arm")
#else ()
#    set(ARCH "x86")
#endif ()

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        ros_rs_msgs
        roslib

        nodelet
        )

# Header path
include_directories(
        $ENV{HOME}/ascend_ddk/${ARCH}/include/opencv4
        $ENV{HOME}/ascend_ddk/${ARCH}/include/
        ${INC_PATH}/acllib/include/
        ./inc/
        ${catkin_INCLUDE_DIRS}
)

catkin_package()

if (target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif ()

# add host lib path
link_directories(
        /usr/local/lib
        ${LIB_PATH}
        $ENV{HOME}/ascend_ddk/${ARCH}/lib/
        $ENV{HOME}/Ascend/
        ${INC_PATH}/atc/lib64
)

#add_executable(main
#        utils.cpp
#        model_process.cpp
#        smobject.cpp
#        object_detect_nms_smobject.cpp
#        main_smobject.cpp
#        )
#
#if (target STREQUAL "Simulator_Function")
#    target_link_libraries(main funcsim)
#else ()
#    target_link_libraries(main ascendcl acl_dvpp stdc++ opencv_highgui opencv_core opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_features2d opencv_videoio protobuf presenteragent)
#endif ()
#
#install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

add_executable(main_ros
        src/utils.cpp
        src/model_process.cpp
        src/object_detect_ros.cpp
        src/main_ros.cpp
        )
target_link_libraries(main_ros
        ascendcl
        acl_dvpp
        stdc++
        ${catkin_LIBRARIES}
        )
add_dependencies(main_ros ros_rs_msgs_generate_messages_cpp)
#install(TARGETS main_ros DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(main_ros_compressed
        src/utils.cpp
        src/model_process.cpp
        src/object_detect_ros.cpp
        src/main_ros_compressedImg.cpp
        )
target_link_libraries(main_ros_compressed
        ascendcl
        acl_dvpp
        stdc++
        ${catkin_LIBRARIES}
        )
add_dependencies(main_ros_compressed ros_rs_msgs_generate_messages_cpp)

add_library(yolo_atlas_nodelet
        src/yolo_atlas_nodelet.cpp
        src/utils.cpp
        src/model_process.cpp
        src/object_detect_ros.cpp
        )
target_link_libraries(yolo_atlas_nodelet
        ascendcl
        acl_dvpp
        stdc++
        ${catkin_LIBRARIES}
        )
add_dependencies(yolo_atlas_nodelet ros_rs_msgs_generate_messages_cpp)