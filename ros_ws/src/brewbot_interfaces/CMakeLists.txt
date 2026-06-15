cmake_minimum_required(VERSION 3.8)
project(brewbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ClassifyText.srv"
  "action/SuggestDrink.action"
  "action/BringDrink.action"
)

ament_package()
