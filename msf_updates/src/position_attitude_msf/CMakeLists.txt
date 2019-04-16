add_executable(position_attitude_sensor main.cpp)

target_link_libraries(position_attitude_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(position_attitude_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
