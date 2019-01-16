# find all files in the current directory
aux_source_directory(. DIR_GEOMETRY_SRCS)
add_library(libgeometry ${DIR_GEOMETRY_SRCS})