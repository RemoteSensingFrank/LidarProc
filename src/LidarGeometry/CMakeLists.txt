# find all files in the current directory
aux_source_directory(. DIR_GEOMETRY_SRCS)
include_directories("../../library")
add_library(libgeometry ${DIR_GEOMETRY_SRCS})