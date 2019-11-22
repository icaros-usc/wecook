add_executable(${PROJECT_NAME}_test_tsr_motion_node src/main.cpp ../../TSRMotionNode.cpp)

target_link_libraries(${PROJECT_NAME}_test_tsr_motion_node
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada
        ${PROJECT_NAME}_ai_external_hauserparabolicsmoother
        ${PROJECT_NAME}_ai_planner_smoother
        ${PROJECT_NAME}_ai_trajectory)

install(TARGETS ${PROJECT_NAME}_test_tsr_motion_node RUNTIME DESTINATION bin)