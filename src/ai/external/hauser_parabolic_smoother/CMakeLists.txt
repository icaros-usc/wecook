add_library("${PROJECT_NAME}_ai_external_hauserparabolicsmoother" STATIC
        DynamicPath.cpp
        ParabolicRamp.cpp
        Timer.cpp
)
target_include_directories("${PROJECT_NAME}_ai_external_hauserparabolicsmoother"
  PUBLIC "."
)
set_target_properties("${PROJECT_NAME}_ai_external_hauserparabolicsmoother"
  PROPERTIES POSITION_INDEPENDENT_CODE TRUE
)
