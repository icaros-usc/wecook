add_executable(${PROJECT_NAME}_test_apriltag_transforms src/main.cpp)

target_link_libraries(${PROJECT_NAME}_test_apriltag_transforms
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libada)

install(TARGETS ${PROJECT_NAME}_test_apriltag_transforms RUNTIME DESTINATION bin)