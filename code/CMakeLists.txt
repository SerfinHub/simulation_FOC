cmake_minimum_required(VERSION 3.15)
project(PLECS VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(components)
add_subdirectory(ports)
add_subdirectory(global)


add_library(${PROJECT_NAME} SHARED main.cpp)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    FuncPorts
    FuncComp
    FuncGlobal
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON

  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"  # .dll (Windows)
)