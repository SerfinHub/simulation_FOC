cmake_minimum_required(VERSION 3.15)
project(PLECS VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(components)
add_subdirectory(ports)
add_subdirectory(global)

add_library(${PROJECT_NAME} SHARED main.cpp)

function(enable_project_warnings target_name)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target_name} PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wshadow
    )
  endif()
endfunction()

enable_project_warnings(FuncPorts)
enable_project_warnings(FuncComp)
enable_project_warnings(FuncGlobal)
enable_project_warnings(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    FuncPorts
    FuncComp
    FuncGlobal
)

target_link_options(${PROJECT_NAME} PRIVATE
  -static-libgcc
  -static-libstdc++
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME "PLECS"
  PREFIX ""

  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
)
