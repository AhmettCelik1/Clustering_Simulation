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


find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono)
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
add_library(clustering_activation_core
            src/Clustering_Activation/Clustering_Activation.cpp
        )
target_compile_features(clustering_activation_core INTERFACE cxx_std_11)


## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(clustering_activation_core
        ${OpenCV_LIBS}
        )

## Specify libraries to link executable targets against
target_link_libraries(clustering_activation_core
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
        ${CMAKE_THREAD_LIBS_INIT}
        
        )


## Declare a cpp library
add_library(euclidean_clustering_core
        src/Euclidean_Clustering/Euclidean_Clustering.cpp
        )

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_compile_features(euclidean_clustering_core INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(euclidean_clustering_core
        ${OpenCV_LIBS}
        )

## Specify libraries to link executable targets against
target_link_libraries(euclidean_clustering_core
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        
        )


## Declare a cpp library
add_library(dbscan_clustering_core
        src/DBscan_Clustering/DBscan_Clustering.cpp
        )

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_compile_features(dbscan_clustering_core INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(dbscan_clustering_core
        ${OpenCV_LIBS}
        )

## Specify libraries to link executable targets against
target_link_libraries(dbscan_clustering_core
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        
        )




## Declare a cpp library
add_library(clustering_visualization_core
        src/Clustering_Visualization/Clustering_Visualization.cpp
        )

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_compile_features(clustering_visualization_core INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(clustering_visualization_core
        ${OpenCV_LIBS}
        )

## Specify libraries to link executable targets against
target_link_libraries(clustering_visualization_core
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        
)




## Declare cpp executables
add_executable(main
        src/main.cpp
        src/Lidar_Utils/Lidar_Utils.cpp
        src/Lidar_Tool_Option/Lidar_Tool_Option.cpp
        src/Clustering_Activation/Clustering_Activation.cpp
        src/K_Means/K_Means.cpp
        src/Euclidean_Clustering/Euclidean_Clustering.cpp
        src/DBscan_Clustering/DBscan_Clustering.cpp
        src/Clustering_Visualization/Clustering_Visualization.cpp
        )

target_compile_features(main INTERFACE cxx_std_11)

target_link_libraries(main
        clustering_activation_core
        utils_core  
        lidar_tool_option_core
        k_means_core
        ${OpenCV_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
        )