cmake_minimum_required(VERSION 3.5)
project(lidar_activation)

add_compile_options(-Wall -Wextra)



## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
set(Boost_USE_STATIC_LIBS ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL 1.2 REQUIRED)

## Declare a cpp library
add_library(utils_core
        src/Lidar_Utils/Lidar_Utils.cpp
        )
target_compile_features(utils_core INTERFACE cxx_std_11)

## Declare a cpp library
add_library(lidar_tool_option_core
        src/Lidar_Tool_Option/Lidar_Tool_Option.cpp
        )
target_compile_features(lidar_tool_option_core INTERFACE cxx_std_11)


## Declare a cpp library
add_library(lidar_activation_core
            src/Lidar_Activation/Lidar_Activation.cpp
        )
target_compile_features(lidar_activation_core INTERFACE cxx_std_11)


## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(lidar_activation_core
        ${OpenCV_LIBS}
        )

## Specify libraries to link executable targets against
target_link_libraries(lidar_activation_core
        ${OpenCV_LIBS}
        )



## Declare a cpp library
add_library(k_means_core
        src/K_Means/K_Means.cpp
        )

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_compile_features(k_means_core INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(k_means_core
        ${OpenCV_LIBS}
        )

## Specify libraries to link executable targets against
target_link_libraries(k_means_core
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        
        )



## Declare cpp executables
add_executable(main
        src/main.cpp
        src/Lidar_Utils/Lidar_Utils.cpp
        src/Lidar_Tool_Option/Lidar_Tool_Option.cpp
        src/Lidar_Activation/Lidar_Activation.cpp
        src/K_Means/K_Means.cpp
        )

target_compile_features(main INTERFACE cxx_std_11)

target_link_libraries(main
        lidar_activation_core
        utils_core  
        lidar_tool_option_core
        k_means_core
        ${OpenCV_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
        )