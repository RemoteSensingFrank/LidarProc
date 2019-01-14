aux_source_directory(. RESEARCH_DIR_SRCS)

include_directories("../../library")

add_library(libresearch ${RESEARCH_DIR_SRCS})
target_link_libraries(libresearch libalgorithm libutility)
