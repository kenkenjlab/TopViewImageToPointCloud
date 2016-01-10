########################################################################

#### User settings #### * Modify here *

set(OPENCV_INC_DIR "C:/opencv3/build/include")
set(OPENCV_LIB_DIR "C:/opencv3/build/x64/vc12/lib")
set(OPENCV_LIBS_DEBUG debug opencv_ts300d.lib debug opencv_world300d.lib)
set(OPENCV_LIBS_RELEASE optimized opencv_ts300.lib optimized opencv_world300.lib)
set(OPENCV_LIBS ${OPENCV_LIBS_DEBUG} ${OPENCV_LIBS_RELEASE})

########################################################################



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

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INC_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${OPENCV_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})


#### Required files ####

set(STRING_UTILITY string_utility.hpp string_utility.cpp)
set(RECIPE recipe.hpp)
set(RECIPE_READER recipe_reader.hpp recipe_reader.cpp ${RECIPE} ${STRING_UTILITY})
set(TOPVIEWIMG_TO_POINTCLOUD_CONVERTER tvimg2pc_converter.hpp tvimg2pc_converter.cpp ${RECIPE})


#### Projects Settings ####

add_executable(TopViewImageToPointCloud main.cpp ${RECIPE_READER} ${TOPVIEWIMG_TO_POINTCLOUD_CONVERTER})
target_link_libraries(TopViewImageToPointCloud ${PCL_LIBRARIES} ${OPENCV_LIBS})