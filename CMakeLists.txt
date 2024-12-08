cmake_minimum_required(VERSION 3.1)

project(FinalProject)

set(CMAKE_CXX_STANDARD 17)

# Find OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found! Please set OpenCV_DIR.")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

# Encoder executable
add_executable(MyEncoder src/MyEncoder.cpp)

# Decoder executable
add_executable(MyDecoder src/MyDecoder.cpp)

# Link libraries
target_link_libraries(MyEncoder ${OpenCV_LIBS})
target_link_libraries(MyDecoder ${OpenCV_LIBS})

# X11 required on Linux
if(UNIX AND NOT APPLE)
    find_package(X11 REQUIRED)
    if(X11_FOUND)
        include_directories(${X11_INCLUDE_DIR})
        target_link_libraries(MyEncoder ${X11_LIBRARIES})
        target_link_libraries(MyDecoder ${X11_LIBRARIES})
    endif()
endif()
