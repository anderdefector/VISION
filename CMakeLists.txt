cmake_minimum_required(VERSION 2.8.3)
project(c_vis)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}	
)

add_executable(VI src/ocv.cpp)
target_link_libraries(VI ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Linea src/linea.cpp)
target_link_libraries(Linea ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Pelota2 src/Pelota_c.cpp)
target_link_libraries(Pelota2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(TEST_VIS src/test_vis.cpp)
target_link_libraries(TEST_VIS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(BASE src/base_l.cpp)
target_link_libraries(BASE ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HIST src/hist.cpp)
#target_link_libraries(HIST ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(TEST src/test.cpp)
target_link_libraries(TEST ${catkin_LIBRARIES})

