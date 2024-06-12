cmake_minimum_required(VERSION 3.5)
project(epuck_ros2_uwb)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic -lbcm2835 -pthread -lwiringPi)
  #add_compile_options(-Wall -Wextra -Wpedantic -pthread)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

#find_package(Threads REQUIRED)
find_library(BCM2835_LIBRARY NAMES bcm2835)
find_library(WIRINGPI_LIBRARY NAMES wiringPi)

add_executable(uwb_pub
  src/uwb_pub.cpp
  src/dwm_lib/dwm_driver/lmh/lmh.c
  src/dwm_lib/dwm_driver/lmh/lmh_spirx.c
  src/dwm_lib/dwm_driver/dwm_api/dwm_api.c
  src/dwm_lib/platform/rpi/hal/hal.c
  src/dwm_lib/platform/rpi/hal/hal_gpio.c
  src/dwm_lib/platform/rpi/hal/hal_log.c
  src/dwm_lib/platform/rpi/hal/hal_spi.c
)
target_include_directories(uwb_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  src/dwm_lib
  src/dwm_lib/dwm_driver/dwm_api
  src/dwm_lib/dwm_driver/lmh
  src/dwm_lib/platform/rpi/hal
)
target_compile_definitions(uwb_pub PUBLIC INTERFACE_NUMBER=1)
#target_link_libraries(uwb_pub ${BCM2835_LIBRARY} ${WIRINGPI_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(uwb_pub ${BCM2835_LIBRARY} ${WIRINGPI_LIBRARY})

ament_target_dependencies(uwb_pub
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS uwb_pub
  EXPORT export_${PROJECT_NAME}
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
