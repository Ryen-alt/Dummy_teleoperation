#pragma once
#include <cstdint>
constexpr uint8_t CDC_OUT_EP = 1, CDC_IN_EP = 0x81, ODRIVE_OUT_EP = 2, ODRIVE_IN_EP = 0x82;
constexpr uint8_t USBD_OK = 0, USBD_BUSY = 1;
inline int hUsbDeviceFS = 0;
inline void USBD_CDC_ReceivePacket(int*, uint8_t) {}
