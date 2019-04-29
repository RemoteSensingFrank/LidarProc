project(PotreeCvtlib)

set(RAPIDJSON_INCLUDE_DIR lib/rapidjson/include)
set(ARGUMENTS_INCLUDE_DIR lib/arguments)
set(LIBLAS_INCLUDE_DIR lib/LASTools/LASlib/inc)
set(LIBLASzip_INCLUDE_DIR lib/LASTools/LASzip/include)

file(GLOB_RECURSE files src/*.cpp include/*.h include/*.hpp )
file(GLOB_RECURSE lib_rapidjson_files ${RAPIDJSON_INCLUDE_DIR}/* )
file(GLOB_RECURSE lib_argument_files ${ARGUMENTS_INCLUDE_DIR}/* )

include_directories(include ${LIBLAS_INCLUDE_DIR})
include_directories(include ${LIBLASzip_INCLUDE_DIR})
include_directories(include ${RAPIDJSON_INCLUDE_DIR})
include_directories(include ${ARGUMENTS_INCLUDE_DIR})

add_subdirectory(./lib/LASTools)

add_library(PotreeCvtlib ${files} ${lib_rapidjson_files} ${lib_argument_files})
target_link_libraries(PotreeCvtlib LASlib)

