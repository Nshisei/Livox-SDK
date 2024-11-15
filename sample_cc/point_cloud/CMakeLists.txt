cmake_minimum_required(VERSION 3.0)

set(point_cloud)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(save_camera_pos save_camera_pos.cpp)
target_link_libraries(save_camera_pos PRIVATE ${PCL_LIBRARIES})

add_executable(vis_all_points vis_all_points.cpp)
target_link_libraries(vis_all_points PRIVATE ${PCL_LIBRARIES})

add_executable(screen_shot screen_shot.cpp)
target_link_libraries(screen_shot PRIVATE ${PCL_LIBRARIES})

add_executable(sampling get_points.cpp sample_lds_lidar.cpp)
target_link_libraries(sampling PRIVATE ${PROJECT_NAME}_static ${PCL_LIBRARIES})
