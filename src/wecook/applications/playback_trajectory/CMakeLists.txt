add_executable(${PROJECT_NAME}_playback_trajectory src/main.cpp)

target_link_libraries(${PROJECT_NAME}_playback_trajectory
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada)

install(TARGETS ${PROJECT_NAME}_playback_trajectory RUNTIME DESTINATION bin)