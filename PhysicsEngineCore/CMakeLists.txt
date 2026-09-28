cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

set(ENGINE_PROJECT_NAME PhysicsEngineCore)


set(CORE_PUBLIC_INCLUDES
src/phx_Body.hpp 
src/phx_Material.hpp
src/phx_World.hpp

src/Colliders/phx_CircleCollider.hpp 
src/Colliders/phx_Collider.hpp
src/Colliders/phx_RectCollider.hpp

src/Collisions/phx_Manifolds.hpp
src/Collisions/phx_NarrowPhase.hpp 
src/Collisions/phx_Solver.hpp 

src/Dynamics/phx_Integrator.hpp

src/Geometry/phx_aabb.hpp 
src/Geometry/phx_matrix.hpp 
src/Geometry/phx_utils.hpp 
src/Geometry/phx_vector.hpp
src/Geometry/phx_matrix.hpp

)

set(CORE_PUBLIC_SOURCES 
src/phx_World.cpp

src/Colliders/phx_CircleCollider.cpp
src/Colliders/phx_RectCollider.cpp 

src/Collisions/phx_NarrowPhase.cpp 
src/Collisions/phx_Solver.cpp 

src/Dynamics/phx_Force.cpp 
src/Dynamics/phx_Gravity.cpp 
src/Dynamics/phx_Integrator.cpp 

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
