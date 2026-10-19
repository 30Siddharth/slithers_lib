cmake_minimum_required(VERSION 3.14)
project(slithers VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
# ament_cmake is optional: the library builds with plain CMake (Isaac Sim,
# MuJoCo, ...) and also as a ROS 2 package when built under colcon.
find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)

option(SLITHERS_BUILD_EXAMPLES "Build the ROS-free example programs" OFF)

# ── Library target ────────────────────────────────────────────────────────────
add_library(slithers SHARED
    src/robot_models.cpp
    src/path_generators.cpp
    src/ik_solver.cpp
    src/planner.cpp
)

target_include_directories(slithers
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(slithers PUBLIC Eigen3::Eigen)

set_target_properties(slithers PROPERTIES
    VERSION   ${PROJECT_VERSION}
    SOVERSION 1
)

# ── Installation ──────────────────────────────────────────────────────────────
install(TARGETS slithers
    EXPORT  slithersTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(EXPORT slithersTargets
    FILE      slithersTargets.cmake
    NAMESPACE slithers::
    DESTINATION lib/cmake/slithers
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/slithersConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/slithersConfig.cmake
    INSTALL_DESTINATION lib/cmake/slithers
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/slithersConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/slithersConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/slithersConfigVersion.cmake
    DESTINATION lib/cmake/slithers
)

# ── Examples ──────────────────────────────────────────────────────────────────
if(SLITHERS_BUILD_EXAMPLES)
    add_executable(slithers_demo examples/slithers_demo.cpp)
    target_link_libraries(slithers_demo PRIVATE slithers)
endif()

# ── ROS 2 / ament (only when available) ───────────────────────────────────────
if(ament_cmake_FOUND)
    ament_export_targets(slithersTargets HAS_LIBRARY_TARGET)
    ament_export_dependencies(Eigen3)
    ament_package()
endif()
