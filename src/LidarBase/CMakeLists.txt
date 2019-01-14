# find all files in the current directory
aux_source_directory(. DIR_LIDARBASE_SRCS)
add_library(libbase ${DIR_LIDARBASE_SRCS})
target_link_libraries(libbase,libgeometry)
