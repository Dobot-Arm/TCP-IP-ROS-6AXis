cmake_minimum_required(VERSION 3.0.2)
project(rosdemo_v4)

set(CMAKE_CXX_STANDARD 14)
#set(JSONCPP_DIR "${PROJECT_SOURCE_DIR}/json/include/nlohmann")

# 添加编译选项
add_compile_options(-std=c++14)

## 添加所需的依赖项
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  message_generation
)

#find_package(jsoncpp REQUIRED PATHS ${JSONCPP_DIR})

## 声明自定义的srv文件
add_service_files(
  FILES
  EnableRobot.srv
  DisableRobot.srv
  ClearError.srv
  GetErrorID.srv
  MovJ.srv
)

## 声明msg所依赖的package
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

#include_directories( ${PROJECT_SOURCE_DIR}/json/include/nlohmann)
## 设置包含路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# # 添加子目录指向 JSON 库源代码的路径
# add_subdirectory(json)

## 声明要编译的源文件
add_executable(${PROJECT_NAME}
         main.cpp
         rosDemoCRV4.cpp
        )

## 链接catkin和你的依赖项
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} nlohmann_json::nlohmann_json)

## 添加依赖项，确保服务文件和头文件被正确地编译
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
