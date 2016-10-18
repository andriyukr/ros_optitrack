cmake_minimum_required(VERSION 2.8.3)
project(controllers)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
#file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sensor_msgs geometry_msgs tf dynamic_reconfigure aruco_mapping)

generate_dynamic_reconfigure_options(cfg/setSIT2FLC.cfg cfg/setDIT2FLC.cfg cfg/setPID.cfg cfg/setTrajectory.cfg cfg/setNoise.cfg cfg/setNoise1.cfg)

catkin_package()

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(include "/opt/ros/kinetic/include" ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(q_learning src/q_learning.cpp)
target_link_libraries(q_learning ${catkin_LIBRARIES})
add_dependencies(q_learning controllers_generate_messages_cpp)

add_executable(se3 src/SE3.cpp)
target_link_libraries(se3 ${catkin_LIBRARIES})
add_dependencies(se3 controllers_generate_messages_cpp)

add_executable(sit2flc src/SIT2FLC.cpp)
target_link_libraries(sit2flc ${catkin_LIBRARIES})
add_dependencies(sit2flc controllers_generate_messages_cpp)

add_executable(dit2flc src/DIT2FLC.cpp)
target_link_libraries(dit2flc ${catkin_LIBRARIES})
add_dependencies(dit2flc controllers_generate_messages_cpp)

add_executable(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
add_dependencies(pid controllers_generate_messages_cpp)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
add_dependencies(trajectory controllers_generate_messages_cpp)

add_executable(noise src/noise.cpp)
target_link_libraries(noise ${catkin_LIBRARIES})
add_dependencies(noise controllers_generate_messages_cpp)

add_executable(logger src/logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})
add_dependencies(logger controllers_generate_messages_cpp)

add_executable(safe_bebop src/safeBebop.cpp)
target_link_libraries(safe_bebop ${catkin_LIBRARIES})
add_dependencies(safe_bebop controllers_generate_messages_cpp)

add_executable(teleop_key src/teleopKey.cpp)
target_link_libraries(teleop_key ${catkin_LIBRARIES})
add_dependencies(teleop_key controllers_generate_messages_cpp)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
add_dependencies(odometry controllers_generate_messages_cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
