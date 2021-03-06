PROJECT(youBot_HelloWorldDemo)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
## check required prerequisites
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/external/cmake_modules")
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)

find_package(Eigen REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})
## build parameters and paths
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
## Set include directories
INCLUDE_DIRECTORIES(
${OODL_YOUBOT_INCLUDE_DIR}
${Boost_INCLUDE_DIR}
${EIGEN_INCLUDE_DIRS}
)
## Compile and link
ADD_EXECUTABLE(youbot_kdl
src/main.cpp
src/YouBotKDLInterface.cpp
)
ADD_EXECUTABLE(interface_youbot_kdl
    src/mainInterface.cpp
src/YouBotKDLInterface.cpp
)

TARGET_LINK_LIBRARIES(youbot_kdl
${OODL_YOUBOT_LIBRARIES}
${Boost_LIBRARIES}
orocos-kdl
pthread
/usr/local/lib/libYouBotDriver.so
)
TARGET_LINK_LIBRARIES(interface_youbot_kdl
${OODL_YOUBOT_LIBRARIES}
${Boost_LIBRARIES}
orocos-kdl
pthread
/usr/local/lib/libYouBotDriver.so
)
