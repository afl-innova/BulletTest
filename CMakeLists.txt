cmake_minimum_required(VERSION 3.18)

project("BulletTest" VERSION 1.0)
message("${PROJECT_NAME} ${PROJECT_VERSION}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/")

find_package(Bullet REQUIRED)

#########################################################
set(Target ${PROJECT_NAME})
set(Source
    src/main.cpp
)

set(Header
)

add_executable(${Target} ${Source} ${Header})

target_compile_features(${Target}
    PRIVATE
        cxx_std_17
)

target_include_directories(${Target}
  PRIVATE
    src/
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(${Target}
    ${BULLET_LIBRARIES}
)
