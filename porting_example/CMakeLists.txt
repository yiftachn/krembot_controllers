set (CMAKE_CXX_STANDARD 11)

set(PROGRAM_NAME "porting_example")

add_definitions(
        -DCONTROLLER_NAME=${PROGRAM_NAME}_controller
        -DCONTROLLER_LABEL="${PROGRAM_NAME}_controller"
        -DKREMBOT_SIM
)

set(SRC_FILES
        krembot.ino.cpp
        my_code/MyCode.cpp)

add_library(${PROGRAM_NAME} MODULE ${SRC_FILES})

target_link_libraries(${PROGRAM_NAME}
        Krembot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

