FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(CGAL REQUIRED)

add_library("${PROJECT_NAME}_ai_sensor" STATIC
        Sensor.cpp
        camera/Camera.cpp
        camera/kinect/Kinect.cpp
        camera/kinect/SimKinect.cpp
        )

include_directories(${OpenCV_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
target_link_libraries("${PROJECT_NAME}_ai_sensor" ${OpenCV_LIBS} CGAL::CGAL)

set_target_properties("${PROJECT_NAME}_ai_sensor"
        PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
