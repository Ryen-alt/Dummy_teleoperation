#ifndef DUMMY_CAN_TRANSPORT_CONTRACT_H
#define DUMMY_CAN_TRANSPORT_CONTRACT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    DUMMY_MOTOR_DIAGNOSTICS_FORMAT_V2 = 0xA2,
    DUMMY_MOTOR_DIAGNOSTICS_FORMAT_OFFSET = 4,
    DUMMY_MOTOR_TX_DROP_OFFSET = 5,
    DUMMY_MOTOR_RX_ERROR_OFFSET = 6,
    DUMMY_MOTOR_BUSOFF_OFFSET = 7,
};

static inline uint8_t DummyCanSaturatingIncrement8(uint8_t value)
{
    return value == UINT8_MAX ? value : (uint8_t) (value + 1U);
}

#ifdef __cplusplus
}
#endif

#endif
