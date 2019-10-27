FILE(GLOB ${PROJECT_NAME}_ai_planner_taskPlanner_source *.cpp)
add_library("${PROJECT_NAME}_ai_planner_taskPlanner" STATIC
        ${${PROJECT_NAME}_ai_planner_taskPlanner_source}
        )
target_include_directories("${PROJECT_NAME}_ai_planner_taskPlanner"
        PUBLIC "."
        )
set_target_properties("${PROJECT_NAME}_ai_planner_taskPlanner"
        PROPERTIES POSITION_INDEPENDENT_CODE TRUE
        )