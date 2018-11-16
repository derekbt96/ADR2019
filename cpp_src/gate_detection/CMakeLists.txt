cmake_minimum_required(VERSION 3.10)
project(gate_detection)
find_package(catkin REQUIRED COMPONENTS
        tensorflow_ros_cpp)

#set(TENSORFLOW_FLAGS "-I/usr/local/include -L/usr/local/lib")
#set(TENSORFLOW_LINKER "-ltensorflow")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TENSORFLOW_FLAGS}")
#set(CMAKE_CXX_STANDARD_LIBRARIES "${CMAKE_CXX_STANDARD_LIBRARIES} ${TENSORFLOW_LINKER}")
#set(CMAKE_CXX_STANDARD 11)

add_executable(gate_detection main.cpp)
add_executable(eager_example eager_example.cpp)