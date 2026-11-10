cmake_minimum_required(VERSION 3.10)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(msol_core LANGUAGES C CXX)
endif()

find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(OpenMP QUIET)

add_library(msol_core STATIC msol.cpp)
add_library(msol::msol ALIAS msol_core)
set_target_properties(msol_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(msol_core PUBLIC cxx_std_14)
target_include_directories(msol_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(msol_core PUBLIC ${PCL_LIBRARIES})
target_compile_options(msol_core PUBLIC ${PCL_DEFINITIONS})

if(TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(msol_core PRIVATE OpenMP::OpenMP_CXX)
endif()

