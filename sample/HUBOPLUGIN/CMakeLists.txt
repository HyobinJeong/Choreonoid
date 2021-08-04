
option(BUILD_MYPLUGIN_HUBOPLUGIN "Building a HUBO plugin \"HuboPlugin\"" ON)



if(BUILD_MYPLUGIN_HUBOPLUGIN)
  set(target HuboController)
	add_cnoid_simple_controller(${target} SHARED HuboPlugin.cpp)
endif()




