set (CMAKE_CXX_STANDARD 11)

#todo user: rename program
set(PROGRAM_NAME "simple_demo")

add_definitions(
        -DCONTROLLER_NAME=${PROGRAM_NAME}_controller
        -DCONTROLLER_LABEL="${PROGRAM_NAME}_controller"
)

set(SRC_FILES
        krembot.ino.cpp)

add_library(${PROGRAM_NAME} MODULE ${SRC_FILES})

target_link_libraries(${PROGRAM_NAME}
        Krembot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
