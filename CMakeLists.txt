
# cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
#
# set(PROJECT_NAME PhysicsEngine)
# project(${PROJECT_NAME})
#
# add_subdirectory(PhysicsEngineCore)
# add_subdirectory(PhysicsEngineEditor)
#
#
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(SDL2 REQUIRED sdl2)
# pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)


cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

project(PhysicsEngine)

find_package(PkgConfig REQUIRED)

pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
pkg_check_modules(SDL2_IMAGE REQUIRED IMPORTED_TARGET SDL2_image)

add_subdirectory(PhysicsEngineCore)
add_subdirectory(PhysicsEngineEditor)
