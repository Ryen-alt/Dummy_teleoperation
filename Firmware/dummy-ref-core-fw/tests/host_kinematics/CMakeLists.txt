cmake_minimum_required(VERSION 3.16)
project(dummy_kinematics_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(dummy_kinematics_host_test
    kinematics_host_test.cpp
    ../../Robot/algorithms/kinematic/6dof_kinematic.cpp
)
target_compile_definitions(dummy_kinematics_host_test PRIVATE DUMMY_KINEMATICS_HOST)
target_include_directories(dummy_kinematics_host_test PRIVATE
    ../../Robot/algorithms/kinematic
    ../../UserApp/protocols
    ../../UserApp/configurations
)
if(MSVC)
    target_compile_options(dummy_kinematics_host_test PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(dummy_kinematics_host_test PRIVATE
        -Wall -Wextra -Wpedantic -Werror -UNDEBUG
    )
endif()

enable_testing()
add_test(NAME dummy_kinematics_host_test COMMAND dummy_kinematics_host_test)
