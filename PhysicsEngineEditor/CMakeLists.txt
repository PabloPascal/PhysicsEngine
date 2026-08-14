# cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
#
# set(PROJECT_NAME PhysicsEngineEditor)
#
# project(${PROJECT_NAME})
#
#
# add_executable(${PROJECT_NAME}
# main.cpp
# )
#
# target_link_libraries(${PROJECT_NAME} PhysicsEngineCore ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARIES})
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)



cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

project(PhysicsEngineEditor)

add_executable(${PROJECT_NAME}
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    PhysicsEngineCore
    PkgConfig::SDL2
    PkgConfig::SDL2_IMAGE
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
