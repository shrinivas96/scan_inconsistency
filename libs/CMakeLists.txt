# Specify the libraries you want to include
add_subdirectory(consist)

add_library(visibility STATIC consist/src/consist/visibility.cpp)
target_include_directories(visibility PUBLIC consist/src/consist)

add_subdirectory(g2o)
add_library(raw_laser STATIC g2o/g2o/types/data/robot_laser.cpp)
# add_library(se2 STATIC g2o/g2o/types/slam2d/se2.cpp)
