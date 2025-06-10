cmake_minimum_required(VERSION 3.9.4)

# For LTO
cmake_policy(SET CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)

# Must be placed before calling project() to take effect
set(CMAKE_C_COMPILER "/usr/bin/clang-12")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")

project(ros_g29_logitech_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Checks for LTO support and enables it (adds -flto=thin and to cmake flags)
include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)

# Set build flags
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -Wextra -march=native -mtune=native -funroll-loops")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -march=native -mtune=native -funroll-loops")
set(CMAKE_C_FLAGS_DEBUG "-g -Wall -Wextra -march=native -mtune=native -funroll-loops")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra -march=native -mtune=native -funroll-loops")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
)

add_message_files(
    FILES
    ForceFeedback.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED
    src/g29_force_feedback.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${catkin_LIBRARIES}
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
