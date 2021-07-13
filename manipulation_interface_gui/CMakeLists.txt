cmake_minimum_required(VERSION 3.1.0)

project(manipulation_interface_gui VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp tf rosparam_utilities manipulation_interface_mongo configuration_msgs manipulation_msgs)
catkin_package()

include_directories(
src
${catkin_INCLUDE_DIRS})

add_executable(manipulation_interface_gui
    src/main_window.ui
    src/main_window.cpp
    src/main.cpp
    src/qnode.cpp
    resources/images.qrc
)

target_link_libraries(manipulation_interface_gui
   ${catkin_LIBRARIES}
   Qt5::Core
   Qt5::Widgets)
