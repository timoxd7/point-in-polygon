cmake_minimum_required(VERSION 3.5)

project(PointInPolygon)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(Example src/example/example.cpp)
add_executable(Example2 src/example/example2.cpp)

target_link_libraries(Example ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
target_link_libraries(Example2 ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

SET(CMAKE_CXX_FLAGS "-std=c++20 -stdlib=libc++")
