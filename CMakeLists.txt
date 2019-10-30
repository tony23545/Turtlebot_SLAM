cmake_minimum_required(VERSION 3.1.3)

project(brick_search)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        image_transport
        cv_bridge
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf2_ros
        std_srvs
        actionlib
        move_base_msgs
        actionlib_msgs
        message_generation
        )

add_message_files(
  FILES
  ImageResult.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

# brick_search
add_executable(brick_search
        src/brick_search.cpp
        )

target_include_directories(brick_search SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_include_directories(brick_search PRIVATE include)

target_link_libraries(brick_search ${OpenCV_LIBS} ${catkin_LIBRARIES})

# stage 2
add_executable(stage2_node
        src/stage2_node.cpp
        )

target_include_directories(stage2_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(stage2_node PRIVATE include)

target_link_libraries(stage2_node ${catkin_LIBRARIES})

# stage 3
add_executable(stage3_node
        src/stage3_node.cpp
        )
add_dependencies(stage3_node ${catkin_EXPORTED_TARGETS})

target_include_directories(stage3_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(stage3_node PRIVATE include)

target_link_libraries(stage3_node ${catkin_LIBRARIES})

# stage 3 plus
add_executable(stage3_plus_node
        src/stage3_plus_node.cpp
        )
add_dependencies(stage3_plus_node ${catkin_EXPORTED_TARGETS})

target_include_directories(stage3_plus_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_include_directories(stage3_plus_node PRIVATE include)

target_link_libraries(stage3_plus_node ${OpenCV_LIBS} ${catkin_LIBRARIES})


# stage 4
add_executable(stage4_node
        src/stage4_node.cpp
        )
add_dependencies(stage4_node ${catkin_EXPORTED_TARGETS})
target_include_directories(stage4_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_include_directories(stage4_node PRIVATE include)

target_link_libraries(stage4_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
