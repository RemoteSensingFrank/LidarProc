aux_source_directory(. UTILITY_DIR_SRCS)
add_definitions(-DLINUX)
add_library(libutility ${UTILITY_DIR_SRCS})
