cmake_minimum_required(VERSION 3.0.2)
project(template)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
    nav_msgs  # 新增：代码中使用了nav_msgs/Odometry
  tf        # 新增：代码中使用了tf/transform_listener.h
  cv_bridge # 新增：代码中使用了cv_bridge转换图像
  message_generation
)

# 新增：查找OpenCV（代码中使用了OpenCV的二维码检测和图像处理）
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}  # 新增：OpenCV头文件目录
  ${PCL_INCLUDE_DIRS}
)


add_executable(template src/template.cpp)
target_link_libraries(template ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  )
