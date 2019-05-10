set(RAPIDJSON_INCLUDE_DIR lib/rapidjson/include/)
set(ARGUMENTS_INCLUDE_DIR lib/arguments)
set(LIBLAS_INCLUDE_DIR lib/LASTools/LASlib/inc)
set(LIBLASzip_INCLUDE_DIR lib/LASTools/LASzip/include)

file(GLOB_RECURSE files src/*.cpp include/*.h include/*.hpp )
file(GLOB_RECURSE lib_rapidjson_files ${RAPIDJSON_INCLUDE_DIR}/* )
file(GLOB_RECURSE lib_argument_files ${ARGUMENTS_INCLUDE_DIR}/* )

find_package(libLAS REQUIRED)
message("liblas required:")
message(${libLAS_INCLUDE_DIRS})

include_directories(include ${libLAS_INCLUDE_DIRS})
include_directories(include ${RAPIDJSON_INCLUDE_DIR})
include_directories(include ${ARGUMENTS_INCLUDE_DIR})

add_library(libpotreecvt ${files} ${lib_rapidjson_files} ${lib_argument_files})
target_link_libraries(libpotreecvt ${libLAS_LIBRARIES})
if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14 -pthread -lstdc++ -lm")
    target_link_libraries(libpotreecvt -lstdc++fs)
endif()