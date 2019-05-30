cmake_minimum_required(VERSION 2.8)
project(LidarProc) 
# Add the source in project root directory 
aux_source_directory(./LidarUnitTest DIRTESTS)
add_definitions(-D_DEBUG)
# Add header file include directories 

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


# Target 
add_executable(LidarProc LidarProc.cpp ${DIRTESTS})  
add_executable(LidarServerExample LidarServer.cpp)

set(lnkLibs libgeometry libbase libalgorithm libutility libresearch libpotreecvt)
set(exlnkLibs libgtest.so libgtest_main.so pthread)

target_link_libraries(LidarProc ${lnkLibs})
target_link_libraries(LidarProc ${exlnkLibs})

target_link_libraries(LidarServerExample ${lnkLibs})
target_link_libraries(LidarServerExample ${exlnkLibs} laszip)

