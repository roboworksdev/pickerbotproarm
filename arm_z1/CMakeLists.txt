cmake_minimum_required(VERSION 3.8)
project(arm_z1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

link_directories(lib)




add_executable(static_arm_transformations src/static_arm_transformations.cpp)



add_executable(z1_ros src/z1_ros.cpp)


ament_target_dependencies(z1_ros rclcpp std_msgs tf2 tf2_ros geometry_msgs)


ament_target_dependencies(static_arm_transformations
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)


target_include_directories(z1_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(z1_ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(z1_ros Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})

#ament_target_dependencies( z1_ros z1_sdk)


install(TARGETS static_arm_transformations
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS z1_ros
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY lib/
  DESTINATION lib
)

ament_package()
