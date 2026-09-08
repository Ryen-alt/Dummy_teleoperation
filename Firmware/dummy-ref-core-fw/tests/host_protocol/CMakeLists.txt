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
    ../../UserApp/protocols/can_feedback_monitor.cpp
    ../../UserApp/protocols/feedback_safety_supervisor.cpp
    ../../UserApp/protocols/feedback_poll_scheduler.cpp
    ../../UserApp/protocols/can_timing_profiler.cpp
    ../../UserApp/protocols/ieee754_finite.hpp
    ../../../can_transport_contract.h
    ../../../can_tx_lifecycle.hpp
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

# Compile the real BSP callbacks/admission/recovery code, not a second copy
# of their state transitions. Stub only the hardware and RTOS boundary.
add_executable(dummy_can_transport_host_test
    can_transport_host_test.cpp
    ../../Bsp/communication/interface_can.cpp
)
target_include_directories(dummy_can_transport_host_test PRIVATE
    can_stubs ../../Bsp/communication ../../UserApp
)
if(MSVC)
    target_compile_options(dummy_can_transport_host_test PRIVATE /W4 /WX /UNDEBUG)
    set_property(TARGET dummy_can_transport_host_test PROPERTY CXX_STANDARD 20)
else()
    target_compile_options(dummy_can_transport_host_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME dummy_can_transport_host_test COMMAND dummy_can_transport_host_test)
