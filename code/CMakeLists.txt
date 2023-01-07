cmake_minimum_required(VERSION 3.0.0)

project(PLECS VERSION 0.1.0)

add_subdirectory(components)
add_subdirectory(ports)
add_subdirectory(global)

ADD_LIBRARY(${PROJECT_NAME} SHARED main.cpp)
target_link_libraries(${PROJECT_NAME} FuncPorts FuncComp FuncGlobal)
set_property(TARGET ${PROJECT_NAME} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/build)


