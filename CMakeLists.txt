cmake_minimum_required(VERSION 3.8)
project(ur5e_ign)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
