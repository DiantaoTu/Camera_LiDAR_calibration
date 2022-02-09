cmake_minimum_required(VERSION 3.4)
project(camera_LiDAR_calibration)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-fopenmp")

IF (CMAKE_BUILD_TYPE MATCHES "Debug")
        MESSAGE(STATUS "CMAKE_BUILD_TYPE = Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
ELSEIF (CMAKE_BUILD_TYPE MATCHES "Release")
        MESSAGE(STATUS "CMAKE_BUILD_TYPE = Release")
ELSE()
        set(CMAKE_BUILD_TYPE "Release")
        MESSAGE(STATUS "Set defalult CMAKE_BUILD_TYPE = Release")
ENDIF()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "find opencv version:" ${OpenCV_VERSION})

include_directories("/usr/local/include/eigen3/")

find_package(Boost REQUIRED COMPONENTS system filesystem program_options serialization)
include_directories(${Boost_INCLUDE_DIRS})
link_directories( ${Boost_LIBRARY_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

# find_package(Glog REQUIRED)
# include_directories(${Glog_INCLUDE_DIRS})

aux_source_directory(./ src_files)

# 主函数
add_executable(${PROJECT_NAME} ${src_files})

target_link_libraries(${PROJECT_NAME} 
                        ${PCL_LIBRARIES}  
                        ${Boost_LIBRARIES} 
                        ${OpenCV_LIBRARIES}
                        ${CERES_LIBRARIES}                
                        glog
                        )


