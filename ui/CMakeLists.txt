find_package(Qt6 REQUIRED COMPONENTS Widgets Charts Test)

set(CMAKE_AUTOMOC ON)

add_library(lc_motion STATIC
  src/safety_gate.cpp
  src/motion_validator.cpp
  src/demo_plant.cpp
)
target_include_directories(lc_motion PUBLIC src)
target_link_libraries(lc_motion PUBLIC pthread)

add_library(lc_widgets STATIC
  src/mainwindow.cpp
  src/chart_panel.cpp
)
target_include_directories(lc_widgets PUBLIC src)
target_link_libraries(lc_widgets PUBLIC Qt6::Widgets Qt6::Charts lc_motion)

add_executable(lc_motor_control src/main.cpp)
target_link_libraries(lc_motor_control PRIVATE lc_widgets)

add_executable(lc_ui_logic_test tests/test_logic.cpp)
target_link_libraries(lc_ui_logic_test PRIVATE Qt6::Test lc_motion)
add_test(NAME lc_ui_logic COMMAND lc_ui_logic_test)

add_executable(lc_ui_window_test tests/test_window.cpp)
target_link_libraries(lc_ui_window_test PRIVATE Qt6::Test lc_widgets)
add_test(NAME lc_ui_window COMMAND lc_ui_window_test)
set_tests_properties(lc_ui_window PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
