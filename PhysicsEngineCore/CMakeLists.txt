cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

set(ENGINE_PROJECT_NAME PhysicsEngineCore)


set(CORE_PUBLIC_INCLUDES
src/phx_vector.hpp
src/phx_circle.hpp
src/calculate_collisions.hpp
src/phx_world.hpp
src/calculate_force.hpp
src/phx_rect.hpp
)

set(CORE_PUBLIC_SOURCES 
src/phx_circle.cpp
src/calculate_collisions.cpp
src/phx_world.cpp
src/calculate_force.cpp
src/phx_rect.cpp
)

set(ENGINE_ALL_SOURCES
    ${CORE_PUBLIC_SOURCES}
    ${CORE_PUBLIC_INCLUDES}
)



add_library(${ENGINE_PROJECT_NAME} STATIC ${ENGINE_ALL_SOURCES})

target_include_directories(${ENGINE_PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)


source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ENGINE_ALL_SOURCES})

target_include_directories(${ENGINE_PROJECT_NAME} PRIVATE src)
target_compile_features(${ENGINE_PROJECT_NAME} PUBLIC cxx_std_17)
