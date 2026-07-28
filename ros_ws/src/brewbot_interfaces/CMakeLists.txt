cmake_minimum_required(VERSION 3.8)
project(brewbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ClassifyText.srv"
  "srv/GetUserState.srv"
  "srv/AskForWater.srv"
  "action/SuggestDrink.action"
  "action/BringDrink.action"
  "action/DispenseDrink.action"
)

ament_package()
