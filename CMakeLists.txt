cmake_minimum_required(VERSION 4.0)
project(BiDirectinalPathTracer)

set(CMAKE_CXX_STANDARD 20)
include_directories("C:/libs")
add_executable(BiDirectinalPathTracer main.cpp
        camera.cpp
        camera.h
        scene.cpp
        scene.h
        geometry.cpp
        geometry.h
        material.cpp
        material.h
        ray.h
        image.cpp
        image.h
        light.cpp
        light.h
)
