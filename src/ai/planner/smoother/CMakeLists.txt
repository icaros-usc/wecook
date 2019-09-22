FILE(GLOB ${PROJECT_NAME}_ai_planner_smoother_source *.cpp)
add_library("${PROJECT_NAME}_ai_planner_smoother" STATIC
        ${${PROJECT_NAME}_ai_planner_smoother_source}
        )
target_include_directories("${PROJECT_NAME}_ai_planner_smoother"
        PUBLIC "."
        )
target_link_libraries(${PROJECT_NAME}_ai_planner_smoother PRIVATE ${PROJECT_NAME}_ai_external_hauserparabolicsmoother)
set_target_properties("${PROJECT_NAME}_ai_planner_smoother"
        PROPERTIES POSITION_INDEPENDENT_CODE TRUE
        )