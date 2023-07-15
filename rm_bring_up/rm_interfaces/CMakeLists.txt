cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IMU/IMUReceive.msg"
  "msg/IMU/IMUSend.msg"
  "msg/Referee/AerialRobotEnergy.msg"
  "msg/Referee/BuffMusk.msg"
  "msg/Referee/BulletRemaining.msg"
  "msg/Referee/DartClientCmd.msg"
  "msg/Referee/DartRemainingTime.msg"
  "msg/Referee/DartStatus.msg"
  "msg/Referee/EventData.msg"
  "msg/Referee/GameResult.msg"
  "msg/Referee/GameRobotHP.msg"
  "msg/Referee/GameRobotPos.msg"
  "msg/Referee/GameRobotState.msg"
  "msg/Referee/GameState.msg"
  "msg/Referee/JudgeSystemData.msg"
  "msg/Referee/PowerHeatData.msg"
  "msg/Referee/RefereeWarning.msg"
  "msg/Referee/RFIDStatus.msg"
  "msg/Referee/RobotHurt.msg"
  "msg/Referee/ShootData.msg"
  "msg/Referee/SupplyProjectileAction.msg"

  "msg/AutoAim/Armor.msg"
  "msg/AutoAim/Armors.msg"
  "msg/AutoAim/Target.msg"
  "msg/AutoAim/DebugLight.msg"
  "msg/AutoAim/DebugLights.msg"
  "msg/AutoAim/DebugArmor.msg"
  "msg/AutoAim/DebugArmors.msg"

  "msg/Common/LowPassData.msg"
  "msg/Common/IMUData.msg"


  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

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
