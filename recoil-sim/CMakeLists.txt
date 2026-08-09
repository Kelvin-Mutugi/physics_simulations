cmake_minimum_required(VERSION 3.15)
project(recoil_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Prefer a local Raylib install when present, then fall back to fetching source.
set(RAYLIB_ROOT "$ENV{HOME}/.local/raylib-install")

find_path(RAYLIB_INCLUDE_DIR raylib.h
    HINTS "${RAYLIB_ROOT}/include"
)

find_library(RAYLIB_LIBRARY
    NAMES raylib libraylib.a
    HINTS "${RAYLIB_ROOT}/lib64" "${RAYLIB_ROOT}/lib"
)

if (RAYLIB_INCLUDE_DIR AND RAYLIB_LIBRARY)
    add_library(raylib STATIC IMPORTED GLOBAL)
    set_target_properties(raylib PROPERTIES
        IMPORTED_LOCATION "${RAYLIB_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${RAYLIB_INCLUDE_DIR}"
    )
    set(raylib_FOUND TRUE)
else()
    message(STATUS "Local raylib not found, fetching from source...")
    find_package(raylib CONFIG QUIET)

    if (NOT raylib_FOUND)
        include(FetchContent)
        set(FETCHCONTENT_QUIET FALSE)
        FetchContent_Declare(
            raylib
            GIT_REPOSITORY https://github.com/raysan5/raylib.git
            GIT_TAG 5.5
            GIT_SHALLOW TRUE
        )
        set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(raylib)
    endif()
endif()

add_executable(recoil_sim Main.cpp)
target_link_libraries(recoil_sim PRIVATE raylib)

# Linux system deps raylib needs at link time
if (UNIX AND NOT APPLE)
    target_link_libraries(recoil_sim PRIVATE m pthread dl rt X11)
endif()