# find all files in the current directory
aux_source_directory(. LIDARPCL_SRCS)
add_library(libAlgPCL ${LIDARPCL_SRCS})