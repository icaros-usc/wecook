add_executable(${PROJECT_NAME}_simple_human_trajectory src/main.cpp)

target_link_libraries(${PROJECT_NAME}_simple_human_trajectory
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libhuman)

install(TARGETS ${PROJECT_NAME}_simple_human_trajectory RUNTIME DESTINATION bin)