#### CMake Settings ####

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0015 OLD)	#// Do not add relative path if the designated path is not exist
add_definitions(-DUNICODE -D_UNICODE)	#// Set Unicode, not MultiBytes


#### Solution Settings ####

project(TopViewImageToPointCloud)
if(MSVC AND NOT PLATFORM EQUAL 64)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
  message(STATUS "MSVC: Enabled large address awareness")
endif()


#### Includes and Libraries ####

set(OPENCV_USE_STATIC_LIBS OFF)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message(STATUS "MSVC: Enabled OpenMP")
endif()

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})



#### Required files ####

set(TOPVIEWIMG_TO_POINTCLOUD_CONVERTER tvimg2pc_converter.hpp tvimg2pc_converter.cpp recipe.hpp)

#### Projects Settings ####

add_executable(TopViewImageToPointCloud main.cpp ${TOPVIEWIMG_TO_POINTCLOUD_CONVERTER})
target_link_libraries(TopViewImageToPointCloud ${PCL_LIBRARIES} ${OpenCV_LIBS})