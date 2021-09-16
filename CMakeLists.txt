cmake_minimum_required(VERSION 3.0.2)
project(optimized_motion_planner)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

catkin_package(
        LIBRARIES
        CATKIN_DEPENDS roscpp rosgraph_msgs
)

# node: start optimized motion planner
cs_add_executable(start_example
        src/example/start_example.cpp
        )

target_link_libraries(start_example
        ${catkin_LIBRARIES}
        stdc++fs
        )

# node: motion planner
find_package(ompl REQUIRED)
if (OMPL_FOUND)
    message("Found OMPL ${OMPL_VERSION}")
    include_directories(${OMPL_INCLUDE_DIRS})
else ()
    message("OMPL not found")
endif ()

cs_add_library(optimized_motion_planning_dependencies
        src/motion_planning/RRTX.cpp
        src/motion_planning/RRTX_node.cpp
        src/optimizer/CHOMP_dynamic.cpp
        src/optimizer/CHOMP_dynamic_trajectory.cpp
        src/optimizer/CHOMP_dynamic_cost.cpp
        )

cs_add_executable(optimized_motion_planning
        src/motion_planning/motion_planning_node.cpp)


target_link_libraries(optimized_motion_planning
        optimized_motion_planning_dependencies
        ${catkin_LIBRARIES}
        stdc++fs
        ompl
        )
# Finish
cs_install()
cs_export()