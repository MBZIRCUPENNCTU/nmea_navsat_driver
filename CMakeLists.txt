cmake_minimum_required(VERSION 2.8.3)
project(nmea_navsat_driver)

find_package(catkin REQUIRED COMPONENTS roslint message_generation std_msgs geometry_msgs sensor_msgs)

add_message_files(FILES
  GpsLocal.msg
  )

catkin_python_setup()

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  )

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs sensor_msgs
  )

install(PROGRAMS
   scripts/nmea_serial_driver
   scripts/nmea_topic_driver
   scripts/nmea_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
roslint_add_test()
