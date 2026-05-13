cmake_minimum_required(VERSION 3.10)
project(main)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
find_package(ZLIB REQUIRED)
add_executable(main
    src/main.cpp
    src/graph_parse.cpp
    src/landmarks.cpp
    src/bi_obj_a_star.cpp
)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(main PRIVATE ZLIB::ZLIB)

add_executable(gen_landmarks
    src/gen_landmarks.cpp
    src/graph_parse.cpp
    src/landmarks.cpp
)
target_include_directories(gen_landmarks PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(gen_landmarks PRIVATE ZLIB::ZLIB)
