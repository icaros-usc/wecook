add_executable(${PROJECT_NAME}_human_rrt src/main.cpp)

target_link_libraries(${PROJECT_NAME}_human_rrt
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libhuman)

install(TARGETS ${PROJECT_NAME}_human_rrt RUNTIME DESTINATION bin)