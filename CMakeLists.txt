cmake_minimum_required(VERSION 3.10)
project(skyweave_shape_controller)
enable_language(C)
enable_language(CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(pinocchio REQUIRED)
find_package(CasADi REQUIRED)


# Find Pinocchio using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)


include_directories(
                    ${CasADI_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${GAZEBO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PINOCCHIO_INCLUDE_DIRS}
                    )

link_directories(
    ${PINOCCHIO_LIBRARY_DIRS}
    ${GAZEBO_LIBRARY_DIRS})

add_definitions(
    ${PINOCCHIO_CFLAGS_OTHER}  
    ${GAZEBO_CXX_FLAGS})

add_library(skyweave_link_tracker SHARED src/skyweave_link_tracker.cpp
    src/gamma_surface.cpp
    src/inverse_kinematics.cpp
    src/state_estimator.cpp
    src/thrusters.cpp
    src/shape_controller.cpp
)

target_link_libraries(skyweave_link_tracker PRIVATE
            casadi::casadi
            ${GAZEBO_LIBRARIES}
            ${EIGEN3_LIBRARIES}
            ${PINOCCHIO_LIBRARIES}
            )
