aux_source_directory(. RESEARCH_DIR_SRCS)
find_package(ZLIB)
if(ZLIB_FOUND)
	message(${ZLIB_VERSION_STRING})
	#include(${ZLIB_INCLUDE_DIR})
endif()

include_directories("../../library")

add_library(libresearch ${RESEARCH_DIR_SRCS})
target_link_libraries(libresearch libalgorithm libutility)
if(ZLIB_FOUND)
	target_link_libraries(libresearch ${ZLIB_LIBRARIES})
endif()
