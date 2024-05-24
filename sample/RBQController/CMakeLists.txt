cmake_minimum_required(VERSION 3.1)

# 프로젝트 이름 설정
project(RBQController)

# Choreonoid 패키지 찾기
find_package(Choreonoid REQUIRED)

# 컴파일 옵션 설정
add_compile_options("-std=c++11")

# Include 및 링크 디렉토리 설정
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

# RBQController 빌드 옵션 설정
option(BUILD_RBQCONTROLLER "Building a RBQ Controller \"RBQController\"" ON)

# RBQController 빌드 설정
if(BUILD_RBQCONTROLLER)
  set(target RBQController)
  add_cnoid_simple_controller(${target} SHARED RBQController.cpp)
endif()

