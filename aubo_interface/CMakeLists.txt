cmake_minimum_required(VERSION 3.5)
project(aubo_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${PROJECT_SOURCE_DIR}/src/dependents/robotSDK/inc)
#include_directories(${PROJECT_SOURCE_DIR}/src/example)
include_directories(${PROJECT_SOURCE_DIR}/src)

#判断操作系统64还是32位 8*8=64bit
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
     message(STATUS "-------64bit------")
     #include_directories(${PROJECT_SOURCE_DIR}/src/dependents/log4cplus/linux_x64/inc)     
     #link_directories(${PROJECT_SOURCE_DIR}/src/dependents/log4cplus/linux_x64/lib)
     link_directories(${PROJECT_SOURCE_DIR}/src/dependents/robotSDK/lib/linux_x64)
else()
     message(STATUS "-------32bit------")
     #include_directories(${PROJECT_SOURCE_DIR}/src/dependents/log4cplus/linux_x32/inc)
     #link_directories(${PROJECT_SOURCE_DIR}/src/dependents/log4cplus/linux_x32/lib)
     link_directories(${PROJECT_SOURCE_DIR}/src/dependents/robotSDK/lib/linux_x32)
endif()

message(status "----------dir list begin---------")

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(status " inc.dir='${dir}'")
endforeach(dir)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
foreach(dir ${dirs})
  message(status " link.dir='${dir}'")
endforeach(dir)

message(status "----------dir list end-----------")

#set(SRC_DIR ${PROJECT_SOURCE_DIR}/src  ${PROJECT_SOURCE_DIR}/src/example)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

message(status "----------src list begin---------")

foreach(dir ${SRC_DIR})
     message(status, "src.dir='${dir}'")
     aux_source_directory(${dir} SRC_LIST)
endforeach(dir)

message(status "---------------------------------")

foreach(f ${SRC_LIST})
     message(status "sr.file='${f}'")     
     endforeach(f)

message(status "----------src list end----------")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aubo_interface_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(${PROJECT_NAME} ${SRC_LIST})
#add_executable(aubo_interface src/main.cpp)

ament_target_dependencies(aubo_interface rclcpp aubo_interface_msgs)

#target_link_libraries(${PROJECT_NAME}  pthread log4cplus auborobotcontroller)
target_link_libraries(${PROJECT_NAME}  pthread auborobotcontroller)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/src/debug) 

message("------cmake completed------")

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/src/dependents/robotSDK/lib/linux_x64/

  DESTINATION lib/${PROJECT_NAME}
)

install (TARGETS
  aubo_interface
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()