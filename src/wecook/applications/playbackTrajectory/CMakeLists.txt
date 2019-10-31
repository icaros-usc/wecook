add_executable(${PROJECT_NAME}_playbackTrajectory src/main.cpp)

target_link_libraries(${PROJECT_NAME}_playbackTrajectory
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada)

install(TARGETS ${PROJECT_NAME}_playbackTrajectory RUNTIME DESTINATION bin)