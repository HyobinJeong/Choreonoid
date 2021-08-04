
option(BUILD_MYPLUGIN_COLROBOTPLUGIN "Building a ColRobot plugin \"HuboPlugin\"" ON)



if(BUILD_MYPLUGIN_COLROBOTPLUGIN)
  set(target ColRobotController)
	add_cnoid_simple_controller(${target} SHARED ColRobotPlugin.cpp)
endif()




