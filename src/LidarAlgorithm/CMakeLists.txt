# find all files in the current directory
aux_source_directory(. DIR_ALGORITHM_SRCS)
find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})

find_package(OpenCV)
if(OpenCV_FOUND)
    add_compile_definitions(_USE_OPENCV_)
    message(STATUS "complie with openCV included")
    include_directories(${OpenCV_DIRS})
else
    message(STATUS "complie with openCV not included")
endif()


add_library(libalgorithm ${DIR_ALGORITHM_SRCS})
target_link_libraries(libalgorithm libbase libgeometry)
target_link_libraries(libalgorithm ${GDAL_LIBRARY})
if(OpenCV_FOUND)
    target_link_libraries(libalgorithm ${OpenCV_LIBS})
endif()
