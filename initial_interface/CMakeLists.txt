


##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.1)
project(initial_interface)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-funroll-loops -Wall -Ofast -fPIC -Dlinux)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS roscpp tf rosparam_utilities mongo_interactions configuration_msgs manipulation_msgs)

catkin_package()  # perché è vuoto?


#link_directories(${catkin_LIBRARY_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.

## This setting causes Qt's "MOC" generation to happen automatically.

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in package.xml
####   rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Quick)  # GUI ??? XXXXX
##############################################################################
# Sections
##############################################################################
add_definitions(-DQT_NO_KEYWORDS)


include_directories(
src
${catkin_INCLUDE_DIRS}
${Qt5Widgets_INCLUDE_DIRS}
${Qt5Core_INCLUDE_DIRS})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/initial_interface/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(initial_interface "main.cpp" "main_window.cpp" "qnode.cpp")
#add_executable(initial_interface ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(initial_interface
                      ${catkin_LIBRARIES}
                      Qt5::Core
                      Qt5::Quick
                      Qt5::Widgets
                  )


#target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
