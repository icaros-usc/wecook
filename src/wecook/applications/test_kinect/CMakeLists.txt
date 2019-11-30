add_executable(${PROJECT_NAME}_test_kinect src/main.cpp)

target_link_libraries(${PROJECT_NAME}_test_kinect
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada
        ${PROJECT_NAME}_ai_external_hauserparabolicsmoother
        ${PROJECT_NAME}_ai_planner_smoother
        ${PROJECT_NAME}_ai_trajectory
        ${PROJECT_NAME}_ai_sensor)

install(TARGETS ${PROJECT_NAME}_test_kinect RUNTIME DESTINATION bin)