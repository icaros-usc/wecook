cmake_minimum_required(VERSION 3.10.2)
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
        AgentMsg.msg
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
find_package(libhuman REQUIRED)
include_directories(${libhuman_INCLUDE_DIRS})

find_package(Z3
        REQUIRED
        CONFIG
        # `NO_DEFAULT_PATH` is set so that -DZ3_DIR has to be passed to find Z3.
        # This should prevent us from accidentally picking up an installed
        # copy of Z3. This is here to benefit Z3's build system when building
        # this project. When making your own project you probably shouldn't
        # use this option.
        # NO_DEFAULT_PATH
        )

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

include_directories(include)

add_subdirectory(src)
