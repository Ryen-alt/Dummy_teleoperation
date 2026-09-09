#pragma once
#include <array>
#include <cstdint>
#include <cassert>
#include "cmsis_os.h"
#include "fibre/protocol.hpp"
#include "joint_space_mapping.hpp"
struct TestHand { float openedAngle = 0, closedAngle = 1, angle = 0.5F; };
struct DummyRobot {
    struct { float a[6]{}; } currentJoints;
    TestHand test_hand;
    TestHand* hand = &test_hand;
    uint32_t AbsoluteJointPositionGeneration() const { return 1U; }
    bool AbsoluteJointPositionValid() const { return true; }
    dummy::protocol::AbsoluteJointSeedResult SeedAbsoluteJointPosition(const std::array<float, 6>&) {
        return dummy::protocol::AbsoluteJointSeedResult::Ok;
    }
};
extern unsigned test_critical_depth;
inline void TestEnterCritical() { ++test_critical_depth; }
inline void TestExitCritical() { assert(test_critical_depth); --test_critical_depth; }
#define taskENTER_CRITICAL() TestEnterCritical()
#define taskEXIT_CRITICAL() TestExitCritical()
inline void __DMB() {}
uint32_t micros();
inline osSemaphoreId sem_usb_tx = nullptr, sem_usb_rx = nullptr;
constexpr uint32_t PROTOCOL_SERVER_TIMEOUT_MS = 10;
constexpr size_t USB_TX_DATA_SIZE = 1024, USB_RX_DATA_SIZE = 1024;
