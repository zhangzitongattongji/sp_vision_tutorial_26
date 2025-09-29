cmake_minimum_required(VERSION 3.11)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED) 
# 创建目标 io
add_library(io STATIC 
    my_camera.cpp
    my_camera.hpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include ${CMAKE_CURRENT_SOURCE_DIR})
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl usb-1.0 yaml-cpp ${OpenCV_LIBS})
