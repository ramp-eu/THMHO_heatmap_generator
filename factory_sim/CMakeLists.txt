cmake_minimum_required(VERSION 3.0.2)
project(factory_sim)

find_package(catkin REQUIRED
    COMPONENTS
        costmap_2d
        nav_msgs
        roscpp
        tf
)

catkin_package(
    CATKIN_DEPENDS nav_msgs costmap_2d roscpp tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(costmap_generator src/costmap_gen.cpp)
target_link_libraries(costmap_generator
  ${catkin_LIBRARIES}
)

add_executable(heatmap_generator src/heatmap_generator.cpp)
target_link_libraries(heatmap_generator
  ${catkin_LIBRARIES}
)

install(TARGETS costmap_generator costmap_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS heatmap_generator heatmap_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world)
