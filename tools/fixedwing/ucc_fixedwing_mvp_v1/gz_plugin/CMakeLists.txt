cmake_minimum_required(VERSION 3.22)

project(UccKinematicsPublisher LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(UccKinematicsPublisher SHARED
  UccKinematicsPublisher.cpp
)

target_link_libraries(UccKinematicsPublisher
  PRIVATE
    gz-msgs10::gz-msgs10
    gz-plugin2::gz-plugin2
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
)

target_compile_options(UccKinematicsPublisher PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
