FILE(GLOB ${PROJECT_NAME}_wecook_source *.cpp)

add_executable(wecook ${${PROJECT_NAME}_wecook_source})

add_dependencies(wecook wecook_generate_messages_cpp)
target_link_libraries(wecook
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada
        ${PROJECT_NAME}_ai_planner_smoother)

install(TARGETS wecook RUNTIME DESTINATION bin)