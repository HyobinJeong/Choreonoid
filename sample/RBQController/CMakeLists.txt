cmake_minimum_required(VERSION 3.1)

# 프로젝트 이름 설정
project(RBQController)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Choreonoid 찾기
find_package(Choreonoid REQUIRED)

# QtWidgets 라이브러리 찾기
find_package(Qt5Widgets REQUIRED)

# Include 디렉토리 설정
include_directories(
  ${CHOREONOID_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# 링크 디렉토리 설정
link_directories(${CHOREONOID_LIBRARY_DIRS})

# RBQController 빌드 옵션 설정
option(BUILD_RBQCONTROLLER "Building a RBQ Controller 'RBQController'" ON)

# RBQController 빌드 설정
if(BUILD_RBQCONTROLLER)
  set(target RBQController)
  add_cnoid_simple_controller(${target} SHARED RBQController.cpp)

  # QtWidgets 라이브러리 링크
  target_link_libraries(${target} Qt5::Widgets)
endif()

