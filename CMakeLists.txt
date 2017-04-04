cmake_minimum_required(VERSION 2.6)
project(aruco_markers)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_PREFIX_PATH "/home/yeshi/opencv-3.1.0/install")
set(OpenCV_FOUND TRUE)
#set(OpenCV_DIR /usr/local/lib)
set(OpenCV_DIR REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version : " ${OpenCV_VERSION})
set(OpenCV_INCLUDE_DIRS "/home/yeshi/opencv-3.1.0/install/include")
message(STATUS "OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS})
#set(OpenCV_LIBS "/home/yeshi/opencv-3.1.0/install/lib")
message(STATUS "OpenCV_LIBS : " ${OpenCV_LIBS})


#set(CMAKE_MODULE_PATH /home/yeshi/aruco-2.0.19/build/)
#find_package(aruco REQUIRED)
#message(STATUS "Aruco Version : " ${aruco_VERSION})
#message(STATUS "Aruco_LIBS : " ${aruco_LIBS})
#message(STATUS "Aruco_INCLUDE_DIRS : " ${aruco_INCLUDE_DIRS})


find_package(Boost REQUIRED)
message(STATUS "BOOST_INCLUDE_DIRS : " ${Boost_INCLUDE_DIRS})
message(STATUS "BOOST_LIBRARIES : " ${Boost_LIBRARIES})

find_package(YARP REQUIRED)
message(STATUS "YARP_INCLUDE_DIRS : " ${YARP_INCLUDE_DIRS})
message(STATUS "YARP_LIBRARIES : " ${YARP_LIBRARIES} )

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(aruco_markers main.cpp)

target_link_libraries(aruco_markers ${OpenCV_LIBS} ${Boost_LIBRARIES} ${YARP_LIBRARIES})
#install(TARGETS aruco_markers RUNTIME DESTINATION bin)