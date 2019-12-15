cmake_minimum_required(VERSION 3.8.0)
project(LidarProc) 
# Add the source in project rootRelease directory 
aux_source_directory(./LidarUnitTest DIRTESTS)
SET(CMAKE_BUILD_TYPE "RELEASE")
#SET(CMAKE_BUILD_TYPE "DEBUG")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_definitions(-D_DEBUG)

find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
include_directories(./LidarPotreeCvt/include)
include_directories(./LidarPotreeCvt/lib/rapidjson/include)
include_directories(./LidarPotreeCvt/lib/arguments)
include_directories(./ ./LidarBase ./LidarGeometry ./LidarAlgorithm ./LidarService ./LidarUtil ./LidarResearch)

# Add block directories 
find_package(libLAS REQUIRED)
add_subdirectory(LidarBase) 
add_subdirectory(LidarGeometry) 
add_subdirectory(LidarAlgorithm)
add_subdirectory(LidarUtil)
add_subdirectory(LidarPotreeCvt)
add_subdirectory(LidarResearch)
add_subdirectory(LidarPCLAlgorithm)
add_subdirectory(LidarService)

# Target 
add_executable(LidarProc LidarProc.cpp)  
add_executable(LidarServer LidarServer.cpp)
add_executable(LidarUnitTest LidarUnitTest.cpp ${DIRTESTS})

set(lnkLibs libgeometry libbase libalgorithm libutility libservice libresearch libpotreecvt libAlgPCL)
set(exlnkLibs libgtest.so libgtest_main.so pthread)

target_link_libraries(LidarServer ${lnkLibs})
target_link_libraries(LidarServer ${exlnkLibs} laszip )

target_link_libraries(LidarProc ${lnkLibs})
target_link_libraries(LidarProc ${exlnkLibs} laszip)


target_link_libraries(LidarUnitTest ${lnkLibs})
target_link_libraries(LidarUnitTest ${exlnkLibs} laszip)

if(PCL_FOUND)
    add_definitions(-D_USE_PCL_)
    file(GLOB PCLLIBS "${PCL_LIBRARY_DIRS}/libpcl_*.so")
    target_link_libraries(LidarProc ${PCLLIBS})
    target_link_libraries(LidarServer ${PCLLIBS})
    target_link_libraries(LidarUnitTest ${PCLLIBS})
endif()