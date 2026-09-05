cmake_minimum_required(VERSION 3.16)
project(geo_driver_index CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_library(geoindex src/driver_index.cpp)
target_include_directories(geoindex PUBLIC include)
target_compile_options(geoindex PRIVATE -Wall -Wextra -Wpedantic)

add_executable(unit_tests tests/test_driver_index.cpp)
target_link_libraries(unit_tests PRIVATE geoindex)
target_compile_options(unit_tests PRIVATE -Wall -Wextra)

add_executable(demo examples/demo.cpp)
target_link_libraries(demo PRIVATE geoindex)
target_compile_options(demo PRIVATE -Wall -Wextra)

add_executable(bench bench/benchmark.cpp)
target_link_libraries(bench PRIVATE geoindex)
target_compile_options(bench PRIVATE -Wall -Wextra -O2)

enable_testing()
add_test(NAME unit COMMAND unit_tests)
