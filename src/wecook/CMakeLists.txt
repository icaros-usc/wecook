FILE(GLOB ${PROJECT_NAME}_wecook_source *.cpp Agents/*.cpp LowLevelMotionNodes/*.cpp PrimitiveTaskGraph/*.cpp TaskGraph/*.cpp)

add_executable(wecook ${${PROJECT_NAME}_wecook_source})

add_dependencies(wecook wecook_generate_messages_cpp)
target_link_libraries(wecook
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada
        libhuman
        ${PROJECT_NAME}_ai_planner_smoother
        ${PROJECT_NAME}_ai_trajectory
        ${PROJECT_NAME}_ai_planner_taskPlanner)

install(TARGETS wecook RUNTIME DESTINATION bin)

add_subdirectory(applications)