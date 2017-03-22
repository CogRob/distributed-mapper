cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(distributed-mapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
include(GtsamTesting)

###########################################################################
# Find Boost
find_package(Boost COMPONENTS serialization filesystem system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
message("Boost Libraries: ${Boost_LIBRARIES}")

###########################################################################
# Distributed Mapper
file(GLOB srcs "src/*.cpp" "src/*.h")
include_directories(src)
add_library (distributed-mapper SHARED ${srcs} ${GTSAM_LIBRARIES})
target_link_libraries(distributed-mapper ${Boost_LIBRARIES} gtsam)

###########################################################################
# Install executable
add_executable (runDistributedMapper  scripts/runDistributedMapper.cpp)
target_link_libraries (runDistributedMapper distributed-mapper ${Boost_LIBRARIES} gtsam)

###########################################################################
# Tests
add_subdirectory(tests)

