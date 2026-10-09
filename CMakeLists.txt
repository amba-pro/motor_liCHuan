cmake_minimum_required(VERSION 3.16)
project(lichuan_ethercat LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED SOEM_ROOT)
  if(DEFINED ENV{SOEM_ROOT} AND NOT "$ENV{SOEM_ROOT}" STREQUAL "")
    set(_soem_default "$ENV{SOEM_ROOT}")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../SOEM/soem/ethercat.h")
    set(_soem_default "${CMAKE_CURRENT_SOURCE_DIR}/../SOEM")
  else()
    message(FATAL_ERROR "Set -DSOEM_ROOT= to a built SOEM 1.4.0 tree.")
  endif()
  set(SOEM_ROOT "${_soem_default}" CACHE PATH "SOEM v1.4.0 source tree")
endif()

set(SOEM_BUILD "${SOEM_ROOT}/build")
set(SOEM_LIB "${SOEM_BUILD}/libsoem.a")

if(NOT EXISTS "${SOEM_LIB}")
  message(FATAL_ERROR "SOEM library not found at ${SOEM_LIB}. Build SOEM first.")
endif()

add_library(lc_e_core
  src/cia402.cpp
  src/ethercat_master.cpp
)
target_include_directories(lc_e_core PUBLIC
  src
  "${SOEM_ROOT}/soem"
  "${SOEM_ROOT}/osal"
  "${SOEM_ROOT}/osal/linux"
  "${SOEM_ROOT}/oshw/linux"
)
target_link_libraries(lc_e_core PUBLIC "${SOEM_LIB}" pthread rt)

add_executable(lc_e_diag src/lc_e_diag.cpp)
target_link_libraries(lc_e_diag PRIVATE lc_e_core)

add_executable(lc_e_control src/lc_e_control.cpp)
target_link_libraries(lc_e_control PRIVATE lc_e_core)

add_executable(lc_e_op_disabled src/lc_e_op_disabled.cpp)
target_link_libraries(lc_e_op_disabled PRIVATE lc_e_core)

add_executable(lc_e_csp_hold src/lc_e_csp_hold.cpp src/telemetry_publisher.cpp)
target_link_libraries(lc_e_csp_hold PRIVATE lc_e_core)

add_executable(lc_e_csp_enable src/lc_e_csp_enable.cpp)
target_link_libraries(lc_e_csp_enable PRIVATE lc_e_core)

add_executable(lc_e_csp_move src/lc_e_csp_move.cpp)
target_link_libraries(lc_e_csp_move PRIVATE lc_e_core)

add_executable(lc_e_trajectory_test src/lc_e_trajectory_test.cpp)
target_include_directories(lc_e_trajectory_test PRIVATE src)
enable_testing()
add_test(NAME lc_e_trajectory COMMAND lc_e_trajectory_test)

install(TARGETS lc_e_diag lc_e_control RUNTIME DESTINATION bin)

find_package(Qt6 QUIET COMPONENTS Widgets Charts Test Network)
if(Qt6_FOUND)
  enable_testing()
  add_subdirectory(ui)
endif()
