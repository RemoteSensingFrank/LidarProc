cmake_minimum_required(VERSION 3.8.0)
project(LidarProc) 
# Add the source in project root directory 
aux_source_directory(./LidarUnitTest DIRTESTS)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_definitions(-D_DEBUG)

find_package(OpenCV)
if(OpenCV_FOUND)
	add_definitions(-D_USE_OPENCV_)
endif()

find_package(PCL)
if(PCL_FOUND)
    add_definitions(-D_USE_PCL_)
    include_directories(${PCL_INCLUDE_DIRS})
endif()

# Add header file include directories 
include_directories("../../library/")
include_directories(./ ./LidarBase ./LidarGeometry ./LidarAlgorithm ./LidarUtil ./LidarResearch)
include_directories(./LidarPotreeCvt/include)
include_directories(./LidarPotreeCvt/lib/rapidjson/include)
include_directories(./LidarPotreeCvt/lib/arguments)

# Add block directories 
find_package(libLAS REQUIRED)
add_subdirectory(LidarBase) 
add_subdirectory(LidarGeometry) 
add_subdirectory(LidarAlgorithm)
add_subdirectory(LidarUtil)
add_subdirectory(LidarPotreeCvt)
add_subdirectory(LidarResearch)
add_subdirectory(LidarPCLAlgorithm)

# Target 
add_executable(LidarProc LidarProc.cpp)  
add_executable(LidarServer LidarServer.cpp)

set(lnkLibs libgeometry libbase libalgorithm libutility libresearch libpotreecvt libAlgPCL)
set(exlnkLibs libgtest.so libgtest_main.so pthread)

target_link_libraries(LidarServer ${lnkLibs})
target_link_libraries(LidarServer ${exlnkLibs} laszip )

target_link_libraries(LidarProc ${lnkLibs})
target_link_libraries(LidarProc ${exlnkLibs} laszip)

if(PCL_FOUND)
    add_definitions(-D_USE_PCL_)
    file(GLOB PCLLIBS "${PCL_LIBRARY_DIRS}/libpcl_*.so")
    target_link_libraries(LidarProc ${PCLLIBS})
endif()