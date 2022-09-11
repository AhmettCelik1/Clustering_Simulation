cmake_minimum_required(VERSION 3.5)
project(lidar_activation)

add_compile_options(-Wall -Wextra)



## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

find_package(OpenCV REQUIRED)

## Declare a cpp library
add_library(utils_core
        src/Lidar_Utils/Lidar_Utils.cpp
        )
target_compile_features(utils_core INTERFACE cxx_std_11)


## Declare a cpp library
add_library(lidar_activation_core
            src/Lidar_Activation/Lidar_Activation.cpp
        )
target_compile_features(lidar_activation_core INTERFACE cxx_std_11)


## Declare cpp executables
add_executable(main
        src/main.cpp
        )

target_compile_features(main INTERFACE cxx_std_11)

target_link_libraries(main
        lidar_activation_core
        utils_core  
        ${OpenCV_LIBS}
        )
