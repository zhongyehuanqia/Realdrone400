cmake_minimum_required(VERSION 3.0.2)
project(uav_vision)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs std_msgs nav_msgs quadrotor_msgs cv_bridge image_transport)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/vision_node.py
  scripts/image_publisher.py
  scripts/camera_publisher.py
  scripts/info_logger.py
  scripts/ground_station.py
  scripts/vision_servo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config images DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
