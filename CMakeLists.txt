cmake_minimum_required(VERSION 3.22)
project(drone_observer VERSION 0.1.0)

# Define install directory paths for JRL package config
include(GNUInstallDirs)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Append robotpkg prefix so CMake can find Pinocchio
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

include(FetchContent)
set(CMAKE_CXX_STANDARD 23)

find_package(Eigen3 3.3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

message(STATUS "Fetching spdlog from GitHub...")
FetchContent_Declare(
        spdlog_git
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG        v1.14.1
)
FetchContent_MakeAvailable(spdlog_git)

# Locate system packages
find_package(pinocchio REQUIRED)
find_package(ompl REQUIRED)

add_executable(drone_observer main.cpp
        include/drone_observer/dynamics/dynamics.hpp
        include/drone_observer/dynamics/rk4_integrator.hpp
        include/drone_observer/estimation/ceres_auto_diff.hpp
        include/drone_observer/control/simple_pid.hpp
        include/visualization/visualizer.hpp
        include/drone_observer/estimation/ghkf_observer.hpp
        include/drone_observer/estimation/ckf_observer.hpp
        include/drone_observer/estimation/ekf_observer.hpp
        include/drone_observer/sensors/sensor.hpp
        include/drone_observer/control/trajectories.hpp
        include/drone_observer/control/trajectories.hpp
        include/drone_observer/control/cascaded_controller.hpp)

target_include_directories(drone_observer PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        include/
)

target_link_libraries(drone_observer PRIVATE
        pinocchio::pinocchio
        ${OMPL_LIBRARIES}
        spdlog::spdlog
        ${Pangolin_LIBRARIES}
        Ceres::ceres
)

target_compile_options(drone_observer PRIVATE -O3 -Wall)

# ─── SATISFY ANY RESIDUAL JRL EXPORT TARGET REQUIREMENT ───
install(TARGETS drone_observer EXPORT drone_observerTargets DESTINATION bin)