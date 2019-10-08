# find all files in the current directory
aux_source_directory(. DIR_LIDARBASE_SRCS)
add_library(libservice ${DIR_LIDARBASE_SRCS})