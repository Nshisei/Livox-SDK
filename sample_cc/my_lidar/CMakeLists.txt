cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME my_lidar)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${DEMO_NAME} main.cpp lds_lidar.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
	${PCL_LIBRARIES}
        )
