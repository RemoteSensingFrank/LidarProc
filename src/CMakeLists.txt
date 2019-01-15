cmake_minimum_required(VERSION 2.8)
project(LidarProc) 
# Add the source in project root directory 
aux_source_directory(. DIRSRCS) 
aux_source_directory(./LidarUnitTest DIRTESTS)
# Add header file include directories 

include_directories(./ ./LidarBase ./LidarGeometry ./LidarAlgorithm ./LidarUtil ./LidarResearch) 


# Add block directories 
add_subdirectory(LidarBase) 
add_subdirectory(LidarGeometry) 
add_subdirectory(LidarAlgorithm)
add_subdirectory(LidarUtil)
add_subdirectory(LidarResearch)

# Target 
add_executable(LidarProc ${DIRSRCS} ${DIRTESTS})  


target_link_libraries(LidarProc libgeometry)
target_link_libraries(LidarProc libbase)
target_link_libraries(LidarProc libalgorithm)
target_link_libraries(LidarProc libutility)
target_link_libraries(LidarProc libresearch)
target_link_libraries(LidarProc libgtest.so libgtest_main.so pthread)
