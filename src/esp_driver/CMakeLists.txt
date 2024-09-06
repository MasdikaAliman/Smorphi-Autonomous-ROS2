cmake_minimum_required(VERSION 3.8)
project(esp_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)






# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include/ArxContainer)
include_directories(include/ArxSmartPtr)
include_directories(include/ArxTypeTraits)
include_directories(include/CRCx)
include_directories(include/DebugLog)
include_directories(include/MsgPack)
include_directories(include/Packetizer)
add_executable(esp_driver src/esp_driver.cpp)
ament_target_dependencies(esp_driver rclcpp geometry_msgs nav_msgs serial tf2_ros tf2_geometry_msgs)
target_include_directories(esp_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(esp_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS esp_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
config
launch
map
DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
