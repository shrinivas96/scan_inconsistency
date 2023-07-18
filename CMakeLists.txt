project(scan_inconsistency)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)

# Add the necessary subdirectories
add_subdirectory(src)
add_subdirectory(libs)

include_directories(${PROJECT_SOURCE_DIR}/libs/consist/src ${PROJECT_SOURCE_DIR}/libs/g2o/g2o)

# not sure where they belong
# target_include_directories(scan_inconsistency ${g2o_INCLUDE_DIRS} ${consist_INCLUDE_DIRS})
# target_link_libraries(scan_main ${g2o_LIBRARIES} ${consist_LIBRARIES})