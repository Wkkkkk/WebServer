cmake_minimum_required(VERSION 2.8.3)
project(server)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        automsgs
        roscpp
        geometry_msgs
        jsk_recognition_msgs
        rospy
        )

find_package(messages REQUIRED)
find_package(OpenSSL)
find_package(Boost 1.66 COMPONENTS date_time filesystem iostreams thread)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
        automsgs
        roscpp
        geometry_msgs
        jsk_recognition_msgs
        rospy
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OPENSSL_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${messages_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS})

file(GLOB SOURCES_HPP "include/*.h")
file(GLOB SOURCES_CPP "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES_HPP} ${SOURCES_CPP})
#target_link_libraries(server_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_LIBRARIES} ${PTHREAD_LIB} pthead dl bz)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENSSL_LIBRARIES}
        dl
        messages)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
