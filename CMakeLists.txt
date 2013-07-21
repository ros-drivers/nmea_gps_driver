cmake_minimum_required(VERSION 2.8.3)
project(nmea_gps_driver)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
	scripts/nmea_gps_driver.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

