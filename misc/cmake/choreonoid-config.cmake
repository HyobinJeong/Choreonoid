# Use the following variables to compile and link against Choreonoid:
# CHOREONOID_FOUND            - True if Choreonoid was found on your system
# CHOREONOID_VERSION_STRING   - A human-readable string containing the version
# CHOREONOID_VERSION_MAJOR    - The major version of Choreonoid
# CHOREONOID_VERSION_MINOR    - The minor version of Choreonoid
# CHOREONOID_VERSION_PATCH    - The patch version of Choreonoid
# CHOREONOID_ROOT_DIR         - The base directory of Choreonoid
# CHOREONOID_DEFINITIONS      - Definitions needed to build with Choreonoid
# CHOREONOID_INCLUDE_DIRS     - List of directories of Choreonoid and it's dependencies
# CHOREONOID_LIBRARY_DIRS     - List of directories of Choreonoid and it's dependencies
# CHOREONOID_LIBRARIES        - List of directories of Choreonoid and it's dependencies

set(CHOREONOID_ROOT_DIR "/usr/local")

set(CHOREONOID_VERSION_STRING "1.7.0")
set(CHOREONOID_VERSION_MAJOR "1")
set(CHOREONOID_VERSION_MINOR "7")
set(CHOREONOID_VERSION_PATCH "0")

set(CHOREONOID_DEFINITIONS -DEIGEN_NO_DEBUG;-DQT_NO_DEBUG;-DQT_NO_KEYWORDS;-DQT_NO_OPENGL_ES_2)
set(CHOREONOID_INCLUDE_DIRS /usr/local/include/choreonoid-1.7;/usr/include/eigen3;/usr/include;/usr/include/x86_64-linux-gnu/qt5/;/usr/include/x86_64-linux-gnu/qt5/QtCore;/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++;/usr/include/x86_64-linux-gnu/qt5/QtGui;/usr/include/x86_64-linux-gnu/qt5/QtWidgets;/usr/include/x86_64-linux-gnu/qt5/QtOpenGL;/usr/include/x86_64-linux-gnu/qt5/QtNetwork;/include;/usr/include/python3.8)
set(CHOREONOID_LIBRARY_DIRS /usr/local/lib;/usr/local/lib/choreonoid-1.7)
set(CHOREONOID_UTIL_LIBRARIES CnoidUtil)
set(CHOREONOID_BASE_LIBRARIES ${CHOREONOID_UTIL_LIBRARIES} CnoidBase)
set(CHOREONOID_BODY_LIBRARIES ${CHOREONOID_UTIL_LIBRARIES} CnoidBody)
set(CHOREONOID_BODY_PLUGIN_LIBRARIES ${CHOREONOID_BASE_LIBRARIES} CnoidBody CnoidBodyPlugin)
set(CHOREONOID_LIBRARIES ${CNOD_BODY_LIBRARIES} ${CHOREONOID_BODY_PLUGIN_LIBRARIES})
set(CHOREONOID_PLUGIN_SUBDIR lib/choreonoid-1.7)
set(CHOREONOID_PLUGIN_DIR /usr/local/lib/choreonoid-1.7)

set(CHOREONOID_ENABLE_PYTHON ON)
set(CHOREONOID_USE_PYBIND11 ON)
set(CHOREONOID_PYTHON_LIBRARIES CnoidPyUtil CnoidPyBase)
set(CHOREONOID_PYTHON_SUBDIR lib/choreonoid-1.7/python)

MARK_AS_ADVANCED(
  CHOREONOID_ROOT_DIR
  CHOREONOID_INCLUDE_DIRS
  CHOREONOID_LIBRARIES
  CHOREONOID_Boost_VERSION
  CHOREONOID_CFLAGS_OTHER
  CHOREONOID_LDFLAGS_OTHER
  CHOREONOID_LIBRARY_SUFFIX
  )

function(add_cnoid_plugin)
  set(target ${ARGV0})
  add_library(${ARGV})
  get_target_property(compile_flags ${target} COMPILE_FLAGS)
  if(compile_flags)
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
  else()
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
  endif()
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}
    ARCHIVE_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}
    RUNTIME_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR})
endfunction()

function(cnoid_make_gettext_mofiles target out_mofiles)
  set(${out_mofiles} "" PARENT_SCOPE)
  file(GLOB pofiles ${CMAKE_CURRENT_SOURCE_DIR}/po/*.po)
  foreach(pofile ${pofiles})
    get_filename_component(lang ${pofile} NAME_WE)
    set(message_location share/locale/${lang}/LC_MESSAGES)
    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${message_location})
    set(version "${CHOREONOID_VERSION_MAJOR}.${CHOREONOID_VERSION_MINOR}")
    set(mofile ${PROJECT_BINARY_DIR}/${message_location}/${target}-${version}.mo)
    add_custom_command(
      OUTPUT ${mofile}
      COMMAND msgfmt -o ${mofile} ${pofile}
      DEPENDS ${pofile}
      )
    list(APPEND mofiles ${mofile})
    install(FILES ${mofile} DESTINATION "share/locale/${lang}/LC_MESSAGES")
  endforeach()
  set(${out_mofiles} ${mofiles} PARENT_SCOPE)
endfunction()

if(CHOREONOID_ENABLE_PYTHON AND CHOREONOID_USE_PYBIND11)
  function(add_cnoid_python_module)
    set(target ${ARGV0})
    string(REGEX REPLACE "^Py(.+)$" "\\1" module ${target})
    set(sources ${ARGV})
    list(REMOVE_AT sources 0)
    add_library(${target} SHARED ${sources})
    if(MSVC)
      set_target_properties(${target} PROPERTIES SUFFIX .pyd)
    else()
      if(ENABLE_GCC_FVISIBILITY_HIDDEN)
	get_target_property(compile_flags ${target} COMPILE_FLAGS)
	if(compile_flags)
	  set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
	else()
	  set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
	endif()
      endif()
      if(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
	if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	  target_compile_options(${target} PRIVATE "-flto;-fno-fat-lto-objects")
	  set(link_lto_flags "-flto")
	elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	  target_compile_options(${target} PRIVATE "-flto=thin")
	  set(link_lto_flags "-flto=thin")
	endif()
	get_target_property(existing_link_flags ${target} LINK_FLAGS)
	if(existing_link_flags STREQUAL existing_link_flags-NOTFOUND)
	  set(existing_link_flags "")
	endif()
	set_target_properties(${target} PROPERTIES LINK_FLAGS "${existing_link_flags} ${link_lto_flags}")
        #add_custom_command(TARGET ${target} POST_BUILD COMMAND strip $<TARGET_FILE:${target}>)
      endif()
    endif()
    set_target_properties(${target}  PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${CHOREONOID_PYTHON_SUBDIR}/cnoid
      LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PYTHON_SUBDIR}/cnoid
      RUNTIME_OUTPUT_NAME ${module}
      LIBRARY_OUTPUT_NAME ${module}
      PREFIX "")
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid
      LIBRARY DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid)
    endfunction()
endif()
