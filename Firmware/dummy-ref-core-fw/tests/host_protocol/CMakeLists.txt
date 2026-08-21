cmake_minimum_required(VERSION 3.16)
project(dummy_protocol_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(dummy_protocol_host_test
    protocol_host_test.cpp
    ../../UserApp/protocols/binary_protocol.cpp
    ../../UserApp/protocols/binary_control_session.cpp
    ../../UserApp/protocols/external_target_executor.cpp
    ../../UserApp/protocols/measured_state_estimator.cpp
)
target_include_directories(dummy_protocol_host_test PRIVATE
    ../../UserApp/protocols
    ../../UserApp/configurations
)
if(MSVC)
    # Release adds NDEBUG, but this standalone test uses assert for checks.
    target_compile_options(dummy_protocol_host_test PRIVATE /W4 /WX /UNDEBUG)
else()
    # Keep the checks active in the Release build required by the acceptance guide.
    target_compile_options(dummy_protocol_host_test PRIVATE
        -Wall -Wextra -Wpedantic -Werror -UNDEBUG
    )
endif()

enable_testing()
add_test(NAME dummy_protocol_host_test COMMAND dummy_protocol_host_test)
