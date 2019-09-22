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
find_package(pr_tsr REQUIRED)
include_directories(${pr_tsr_INCLUDE_DIRS})
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(include)

add_subdirectory("src/external/hauser_parabolic_smoother")

add_executable(wecook
        src/main.cpp
        src/TaskManager.cpp
        include/wecook/TaskManager.h
        include/wecook/Task.h
        include/wecook/Action.h
        include/wecook/Agent.h
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
        include/wecook/tsr/spoon.h
        src/ActionPlanner.cpp
        include/wecook/tsr/food.h
        include/wecook/tsr/pose.h
        include/wecook/MotionNode.h
        include/wecook/TSRMotionNode.h
        include/wecook/ConfMotionNode.h
        include/wecook/LinearDeltaMotionNode.h
        include/wecook/GrabMotionNode.h
        src/TSRMotionNode.cpp
        src/ConfMotionNode.cpp
        src/GrabMotionNode.cpp
        src/LinearDeltaMotionNode.cpp
        include/wecook/strutils.h
        include/wecook/TSRMotionwithConstraintNode.h
        src/TSRMotionwithConstraintNode.cpp
        include/wecook/Container.h
        include/wecook/ConnMetadata.h
        src/Container.cpp
        include/wecook/ContainingMap.h
        src/ContainingMap.cpp
        include/wecook/ConnMotionNode.h
        src/ConnMotionNode.cpp
        include/wecook/PreCondition.h
        include/wecook/TSRPreCondition.h
        include/wecook/ConfPreCondition.h
        include/wecook/AngularDeltaMotionNode.h
        src/AngularDeltaMotionNode.cpp
        include/wecook/dart/FixedFrame.h
        include/wecook/DeltaMotionNode.h
        src/DeltaMotionNode.cpp
        include/wecook/IKMotionNode.h
        src/IKMotionNode.cpp
        include/wecook/GravityMotionNode.h
        src/GravityMotionNode.cpp
        include/wecook/GrabPreCondition.h
        include/wecook/ConnPreCondition.h
        include/wecook/FakeMotionNode.h
        src/FakeMotionNode.cpp
        include/wecook/TSRIKNode.h
        src/TSRIKNode.cpp
        include/wecook/TaskGraph.h
        src/TaskGraph.cpp
        include/wecook/ActionNode.h
        include/wecook/PrimitiveTaskGraph.h
        include/wecook/PrimitiveActionNode.h
        include/wecook/PrimitiveGraspNode.h
        include/wecook/Human.h
        include/wecook/Robot.h
        include/wecook/PrimitiveEngageNode.h
        include/wecook/PrimitiveActuateNode.h
        include/wecook/PrimitivePlaceNode.h
        include/wecook/RelativeIKMotionNode.h
        src/RelativeIKMotionNode.cpp
        src/PrimitiveTaskGraph.cpp
        src/PrimitiveActionNode.cpp
        include/wecook/PrimitiveActionExecutor.h
        src/PrimitiveActionExecutor.cpp
        src/PrimitiveGraspNode.cpp
        src/PrimitiveEngageNode.cpp
        src/PrimitivePlaceNode.cpp
        src/PrimitiveActuateNode.cpp
        src/Robot.cpp src/Human.cpp
        include/wecook/ObjectMgr.h
        src/ObjectMgr.cpp
        include/wecook/TaskExecutorThread.h
        src/TaskExecutorThread.cpp
        include/wecook/planner/SmootherHelpers.h
        src/planner/SmootherHelpers.cpp src/planner/SmootherUtil.h src/planner/SmootherUtil.cpp src/planner/SimpleDynamicPath.h src/planner/SimpleDynamicPath.cpp include/wecook/MetaActuateInfo.h)
add_dependencies(wecook wecook_generate_messages_cpp)
target_link_libraries(wecook
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada
        ${PROJECT_NAME}_external_hauserparabolicsmoother)

install(TARGETS wecook RUNTIME DESTINATION bin)

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