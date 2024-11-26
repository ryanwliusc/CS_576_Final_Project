cmake_minimum_required(VERSION 3.1)

project(FinalProject)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})

# Encoder executable
add_executable(MyEncoder src/MyEncoder.cpp)

# Decoder executable
add_executable(MyDecoder src/MyDecoder.cpp)

# X11 required on Linux
if(UNIX AND NOT APPLE)
    find_package(X11 REQUIRED)
endif()

# Link libraries
target_link_libraries(MyDecoder ${OpenCV_LIBS})