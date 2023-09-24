project(kf)
cmake_minimum_required(VERSION 3.10)

add_subdirectory(eigen/pybind11)
include_directories(eigen/eigen-3.4.0)

pybind11_add_module(kf_cpp wrappers.cpp)

