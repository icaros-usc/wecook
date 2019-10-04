FILE(GLOB ${PROJECT_NAME}_ai_trajectory_source *.cpp)
add_library("${PROJECT_NAME}_ai_trajectory" STATIC
        ${${PROJECT_NAME}_ai_trajectory_source}
        )
target_include_directories("${PROJECT_NAME}_ai_trajectory"
        PUBLIC "."
        )
target_link_libraries("${PROJECT_NAME}_ai_trajectory" PRIVATE ${PROJECT_NAME}_ai_external_hauserparabolicsmoother)
set_target_properties("${PROJECT_NAME}_ai_trajectory"
        PROPERTIES POSITION_INDEPENDENT_CODE TRUE
        )