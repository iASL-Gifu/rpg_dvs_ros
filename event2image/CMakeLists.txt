cmake_minimum_required(VERSION 2.8.3)
project(events2image)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(events2image
  src/events2image.cpp
)

# make the nodelet into a library

# link the executable to the necessary libs
target_link_libraries(events2image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# install the executable
install(
  TARGETS events2image 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cs_install()
cs_export()
