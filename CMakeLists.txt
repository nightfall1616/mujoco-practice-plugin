cmake_minimum_required(VERSION 3.20)

project(ConstantForcePlugin)

find_package(mujoco REQUIRED)

add_library(
    constant_force_plugin
    SHARED
    plugin.cc)

target_link_libraries(
    constant_force_plugin
    PRIVATE
    mujoco::mujoco)