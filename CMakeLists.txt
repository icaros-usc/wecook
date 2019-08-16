cmake_minimum_required(VERSION 3.13.2)
project(wecook)

set(CMAKE_CXX_STANDARD 14)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED
        COMPONENTS
            tf
            tf_conversions
            image_transport
            cv_bridge
            message_generation)
include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
        FILES
        TaskMsg.msg
        ActionMsg.msg
        SceneMsg.msg
        ObjectMsg.msg
        ContainingMsg.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs)

find_package(DART 6.6.2 REQUIRED)
include_directories(${DART_INCLUDE_DIRS})
find_package(tf_conversions REQUIRED)
include_directories(${tf_conversions_INCLUDE_DIRS})
find_package(aikido 0.3.0 REQUIRED
        COMPONENTS
            common
            constraint
            control
            control_ros
            distance
            io
            perception
            planner
            planner_kunzretimer
            planner_ompl
            planner_parabolic
            planner_vectorfield
            planner_kunzretimer
            robot
            rviz
            statespace)
include_directories(${aikido_INCLUDE_DIRS})
find_package(libada REQUIRED)
include_directories(${libada_INCLUDE_DIRS})
find_package(pr_assets REQUIRED)
include_directories(${pr_assets_INCLUDE_DIRS})
find_package(pr_tsr REQUIRED)
include_directories(${pr_tsr_INCLUDE_DIRS})
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(include)

add_executable(wecook
        src/main.cpp
        src/TaskManager.cpp
        include/wecook/TaskManager.h
        include/wecook/Task.h
        include/wecook/Action.h
        include/wecook/Robot.h
        src/Robot.cpp
        include/wecook/World.h
        src/World.cpp
        include/wecook/Object.h
        include/wecook/utils.h
        include/wecook/ActionPlanner.h
        include/wecook/tsr/bowl.h
        include/wecook/tsr/fork.h
        include/wecook/tsr/knife.h
        include/wecook/tsr/plate.h
        include/wecook/tsr/pot.h
        include/wecook/tsr/spoon.h src/ActionPlanner.cpp include/wecook/tsr/food.h include/wecook/tsr/pose.h include/wecook/MotionPlanner.h include/wecook/TSRMotionPlanner.h include/wecook/ConfMotionPlanner.h include/wecook/LinearDeltaMotionPlanner.h include/wecook/GrabMotionPlanner.h src/TSRMotionPlanner.cpp src/ConfMotionPlanner.cpp src/GrabMotionPlanner.cpp src/LinearDeltaMotionPlanner.cpp include/wecook/strutils.h include/wecook/TSRMotionwithConstraintPlanner.h src/TSRMotionwithConstraintPlanner.cpp include/wecook/Container.h include/wecook/ConnMetadata.h src/Container.cpp include/wecook/ContainingMap.h src/ContainingMap.cpp include/wecook/ConnMotionPlanner.h src/ConnMotionPlanner.cpp include/wecook/PreCondition.h include/wecook/TSRPreCondition.h include/wecook/ConfPreCondition.h include/wecook/AngularDeltaMotionPlanner.h src/AngularDeltaMotionPlanner.cpp include/wecook/dart/FixedFrame.h include/wecook/DeltaMotionPlanner.h src/DeltaMotionPlanner.cpp include/wecook/IKMotionPlanner.h src/IKMotionPlanner.cpp include/wecook/GravityMotionPlanner.h src/GravityMotionPlanner.cpp)
add_dependencies(wecook wecook_generate_messages_cpp)
target_link_libraries(wecook
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada)

install(TARGETS wecook RUNTIME DESTINATION bin)

#catkin_install_python(PROGRAMS scripts/ex_task1.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#include(ClangFormat)
#clang_format_setup(VERSION 3.8)
#
#if (CLANG_FORMAT_EXECUTABLE)
#        file(GLOB_RECURSE ALL_SOURCE_FILES
#                LIST_DIRECTORIES false
#                src/*.cpp
#                src/*.h)
#        clang_format_add_sources(${ALL_SOURCE_FILES})
#        clang_format_add_targets()
#endif()