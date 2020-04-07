add_executable(${PROJECT_NAME}_test_apriltag src/main.cpp)

target_link_libraries(${PROJECT_NAME}_test_apriltag
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada)

install(TARGETS ${PROJECT_NAME}_test_apriltag RUNTIME DESTINATION bin)