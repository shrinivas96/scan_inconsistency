add_executable(scan_main main.cpp)


# does not work evem when this line in uncommented
# target_link_libraries(scan_main ${g2o_LIBRARIES} ${consist_LIBRARIES})
target_link_libraries(scan_main visibility raw_laser se2)