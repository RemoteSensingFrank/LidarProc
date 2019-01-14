cmake_minimum_required(VERSION 2.8)
project(LidarProc) 
# Add the source in project root directory 
aux_source_directory(. DIRSRCS) 
# Add header file include directories 
include_directories(./ ./LidarBase ./LidarGeometry ./LidarAlgorithm) 
# Add block directories 
add_subdirectory(LidarBase) 
add_subdirectory(LidarGeometry) 
add_subdirectory(LidarAlgorithm)
# Target 
add_executable(LidarProc ${DIRSRCS}) 
target_link_libraries(LidarProc libgeometry libbase libalgorithm)
