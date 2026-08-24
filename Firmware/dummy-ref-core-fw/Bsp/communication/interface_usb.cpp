#include "common_inc.h"
#include "ascii_processor.hpp"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "interface_usb.hpp"
#include "protocols/binary_control_session.hpp"
#include "protocols/binary_control_bridge.hpp"
#include "protocols/binary_protocol.hpp"
#include "protocols/binary_state_bridge.hpp"
#include "protocols/monotonic_micros.hpp"
#include "configurations/robot_config_generated.hpp"

#include <cstring>

osThreadId_t usbServerTaskHandle;
USBStats_t usb_stats_ = {0};

class USBSender : public PacketSink
{
public:
    USBSender(uint8_t endpoint_pair, const osSemaphoreId &sem_usb_tx)
        : endpoint_pair_(endpoint_pair), sem_usb_tx_(sem_usb_tx)
    {}

    int process_packet(const uint8_t *buffer, size_t length) override
    {
        // cannot send partial packets
        if (length > USB_TX_DATA_SIZE)
            return -1;
        // wait for USB interface to become ready
        if (osSemaphoreAcquire(sem_usb_tx_, PROTOCOL_SERVER_TIMEOUT_MS) != osOK)
        {
            // If the host resets the device it might be that the TX-complete handler is never called
            // and the sem_usb_tx_ semaphore is never released. To handle this we just override the
            // TX buffer if this wait times out. The implication is that the channel is no longer lossless.
            // TODO: handle endpoint reset properly
            usb_stats_.tx_overrun_cnt++;
        }

        // transmit packet
        uint8_t status = CDC_Transmit_FS(const_cast<uint8_t *>(buffer), length, endpoint_pair_);
        if (status != USBD_OK)
        {
            osSemaphoreRelease(sem_usb_tx_);
            return -1;
        }
        usb_stats_.tx_cnt++;

        return 0;
    }

private:
    uint8_t endpoint_pair_;
    const osSemaphoreId &sem_usb_tx_;
};

// Note we could have independent semaphores here to allow concurrent transmission
USBSender usb_packet_output_cdc(CDC_OUT_EP, sem_usb_tx);
USBSender usb_packet_output_native(ODRIVE_OUT_EP, sem_usb_tx);

class TreatPacketSinkAsStreamSink : public StreamSink
{
public:
    TreatPacketSinkAsStreamSink(PacketSink &output) : output_(output)
    {
        channelType = CHANNEL_TYPE_USB;
    }

    int process_bytes(const uint8_t *buffer, size_t length, size_t *processed_bytes)
    {
        // Loop to ensure all bytes get sent
        while (length)
        {
            size_t chunk = length < USB_TX_DATA_SIZE ? length : USB_TX_DATA_SIZE;
            if (output_.process_packet(buffer, chunk) != 0)
                return -1;
            buffer += chunk;
            length -= chunk;
            if (processed_bytes)
                *processed_bytes += chunk;
        }
        return 0;
    }

    size_t get_free_space()
    { return SIZE_MAX; }



private:
    PacketSink &output_;
} usb_stream_output(usb_packet_output_cdc);

// This is used by the printf feature. Hence the above statics, and below seemingly random ptr (it's externed)
// TODO: less spaghetti code
StreamSink *usbStreamOutputPtr = &usb_stream_output;

BidirectionalPacketBasedChannel usb_channel(usb_packet_output_native);

namespace
{
dummy::protocol::SessionConfig MakeBinarySessionConfig()
{
    dummy::protocol::SessionConfig config{};
    config.config_sha256 = dummy::generated_config::kConfigSha256;
    config.joint_min_rad = dummy::generated_config::kJointMinRad;
    config.joint_max_rad = dummy::generated_config::kJointMaxRad;
    config.max_velocity_rad_s = dummy::generated_config::kMaxVelocityRadS;
    config.hardware_parameters_verified =
        dummy::generated_config::kHardwareParametersVerified &&
        dummy::generated_config::kExternalTargetExecutionReady;
    config.max_lease_ms = dummy::generated_config::kLeaseTimeoutMs;
    config.max_target_ttl_ms = dummy::generated_config::kTargetTtlMs;
    return config;
}

dummy::protocol::StreamDecoder binary_decoder;
dummy::protocol::ControlSession binary_session(MakeBinarySessionConfig(), "dummy-ref-v1.12");
dummy::protocol::FeedbackSafetyOutput binary_safety_telemetry{};
dummy::protocol::MonotonicMicros32 binary_monotonic_micros;
uint64_t binary_last_state_us = 0;
uint32_t binary_state_sequence = 0;
bool cdc_binary_frame_active = false;
bool binary_state_stream_enabled = false;

uint64_t BinaryMonotonicMicros()
{
    return binary_monotonic_micros.Extend(micros());
}

} // namespace

namespace dummy::protocol
{

uint64_t BinaryControlMonotonicMicros()
{
    taskENTER_CRITICAL();
    const uint64_t now_us = BinaryMonotonicMicros();
    taskEXIT_CRITICAL();
    return now_us;
}

BinaryControlSnapshot ReadBinaryControlSnapshot(uint64_t now_us)
{
    BinaryControlSnapshot snapshot{};
    taskENTER_CRITICAL();
    binary_session.Tick(now_us);
    snapshot.mode = binary_session.mode();
    snapshot.hello_valid = binary_session.hello_valid();
    snapshot.lease_active = binary_session.lease_active();
    const ActiveTarget& active = binary_session.active_target();
    snapshot.target.position = active.target;
    snapshot.target.max_velocity_rad_s = active.max_velocity;
    snapshot.target.sequence = active.sequence;
    snapshot.target.valid = active.valid;
    taskEXIT_CRITICAL();
    return snapshot;
}

void MarkBinaryTargetApplied(uint32_t sequence)
{
    taskENTER_CRITICAL();
    binary_session.MarkTargetApplied(sequence);
    taskEXIT_CRITICAL();
}

void ApplyBinarySafetyOutcome(const FeedbackSafetyOutput& safety)
{
    taskENTER_CRITICAL();
    binary_safety_telemetry = safety;
    if (safety.fault_bits != 0)
        binary_session.SetFault(safety.fault_bits);
    else if (safety.hold_reason_bits != 0)
        binary_session.RequestSafetyHold(safety.hold_reason_bits);
    taskEXIT_CRITICAL();
}

FeedbackSafetyOutput ReadBinarySafetyTelemetry()
{
    taskENTER_CRITICAL();
    const FeedbackSafetyOutput output = binary_safety_telemetry;
    taskEXIT_CRITICAL();
    return output;
}

bool BinaryControlLeaseActive()
{
    taskENTER_CRITICAL();
    const bool active = binary_session.lease_active();
    taskEXIT_CRITICAL();
    return active;
}

} // namespace dummy::protocol

namespace
{

void SendBinaryPacket(dummy::protocol::Packet& packet, uint64_t now_us)
{
    std::array<uint8_t, 600> encoded{};
    packet.header.sender_time_us = now_us;
    const size_t length = dummy::protocol::EncodePacket(packet, encoded.data(), encoded.size());
    if (length != 0)
        usb_stream_output.process_bytes(encoded.data(), length, nullptr);
}

void ProcessBinaryBytes(const uint8_t* data, size_t length, uint64_t now_us)
{
    for (size_t index = 0; index < length; ++index)
    {
        dummy::protocol::Packet request{};
        if (!binary_decoder.Feed(data[index], request))
            continue;
        taskENTER_CRITICAL();
        auto result = binary_session.Process(request, now_us);
        taskEXIT_CRITICAL();
        SendBinaryPacket(result.response, now_us);
    }
    binary_state_stream_enabled = binary_session.hello_valid();
}

void MaybeSendBinaryState(uint64_t now_us)
{
    constexpr uint64_t kStatePeriodUs = 20000; // 50 Hz, outside the 200 Hz control task.
    if (!binary_state_stream_enabled || !binary_session.hello_valid() ||
        now_us - binary_last_state_us < kStatePeriodUs)
        return;
    binary_last_state_us = now_us;
    const auto safety = dummy::protocol::ReadBinarySafetyTelemetry();
    const auto measurement = dummy::protocol::ReadRobotStateForBinaryProtocol(
        now_us, safety);
    taskENTER_CRITICAL();
    const auto state = binary_session.MakeState(
        measurement.position, measurement.velocity, measurement.validity,
        now_us, safety);
    taskEXIT_CRITICAL();
    dummy::protocol::Packet packet{};
    packet.header.message_type = static_cast<uint8_t>(dummy::protocol::MessageType::State);
    packet.header.session_id = binary_session.telemetry_session_id();
    packet.header.sequence = ++binary_state_sequence;
    packet.header.payload_length = sizeof(state);
    std::memcpy(packet.payload.data(), &state, sizeof(state));
    SendBinaryPacket(packet, now_us);
}
} // namespace


struct USBInterface
{
    uint8_t *rx_buf = nullptr;
    uint32_t rx_len = 0;
    bool data_pending = false;
    uint8_t out_ep;
    uint8_t in_ep;
    USBSender &usb_sender;
};

// Note: statics make this less modular.
// Note: we use a single rx semaphore and loop over data_pending to allow a single pump loop thread
static USBInterface CDC_interface = {
    .rx_buf = nullptr,
    .rx_len = 0,
    .data_pending = false,
    .out_ep = CDC_OUT_EP,
    .in_ep = CDC_IN_EP,
    .usb_sender = usb_packet_output_cdc,
};
static USBInterface ODrive_interface = {
    .rx_buf = nullptr,
    .rx_len = 0,
    .data_pending = false,
    .out_ep = ODRIVE_OUT_EP,
    .in_ep = ODRIVE_IN_EP,
    .usb_sender = usb_packet_output_native,
};

static void UsbServerTask(void *ctx)
{
    (void) ctx;

    for (;;)
    {
        // const uint32_t usb_check_timeout = 1; // ms
        osStatus sem_stat = osSemaphoreAcquire(sem_usb_rx, 20);
        if (sem_stat == osOK)
        {
            usb_stats_.rx_cnt++;

            // CDC Interface
            if (CDC_interface.data_pending)
            {
                CDC_interface.data_pending = false;

                const bool starts_binary = CDC_interface.rx_len > 0 && CDC_interface.rx_buf[0] == 0x06;
                const bool starts_ascii = CDC_interface.rx_len > 0 &&
                    CDC_interface.rx_buf[0] >= 0x20 && CDC_interface.rx_buf[0] <= 0x7e;
                const bool binary_lease_active = dummy::protocol::BinaryControlLeaseActive();
                if (starts_ascii && binary_lease_active && !cdc_binary_frame_active)
                {
                    // The maintenance channel cannot take ownership while a
                    // binary control lease exists. Drop the text command.
                }
                else if (cdc_binary_frame_active ||
                    (!starts_ascii && (binary_state_stream_enabled || starts_binary)))
                {
                    ProcessBinaryBytes(
                        CDC_interface.rx_buf, CDC_interface.rx_len,
                        dummy::protocol::BinaryControlMonotonicMicros());
                    cdc_binary_frame_active =
                        CDC_interface.rx_len > 0 && CDC_interface.rx_buf[CDC_interface.rx_len - 1] != 0;
                }
                else
                {
                    // Returning to the maintenance protocol also stops binary telemetry.
                    binary_state_stream_enabled = false;
                    ASCII_protocol_parse_stream(CDC_interface.rx_buf, CDC_interface.rx_len, usb_stream_output);
                }
                USBD_CDC_ReceivePacket(&hUsbDeviceFS, CDC_interface.out_ep);  // Allow next packet
            }

            // Native Interface
            if (ODrive_interface.data_pending)
            {
                ODrive_interface.data_pending = false;
                usb_channel.process_packet(ODrive_interface.rx_buf, ODrive_interface.rx_len);
                USBD_CDC_ReceivePacket(&hUsbDeviceFS, ODrive_interface.out_ep);  // Allow next packet
            }
        }
        MaybeSendBinaryState(dummy::protocol::BinaryControlMonotonicMicros());
    }
}

// Called from CDC_Receive_FS callback function, this allows the communication
// thread to handle the incoming data
void usb_rx_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair)
{
    USBInterface *usb_iface;
    if (endpoint_pair == CDC_interface.out_ep)
    {
        usb_iface = &CDC_interface;
    } else if (endpoint_pair == ODrive_interface.out_ep)
    {
        usb_iface = &ODrive_interface;
    } else
    {
        return;
    }

    // We don't allow the next USB packet until the previous one has been processed completely.
    // Therefore it's safe to write to these vars directly since we know previous processing is complete.
    usb_iface->rx_buf = buf;
    usb_iface->rx_len = len;
    usb_iface->data_pending = true;
    osSemaphoreRelease(sem_usb_rx);
}


const osThreadAttr_t usbServerTask_attributes = {
    .name = "UsbServerTask",
    .stack_size = 4096,
    .priority = (osPriority_t) osPriorityNormal,
};

void StartUsbServer()
{
    // Start USB communication thread
    usbServerTaskHandle = osThreadNew(UsbServerTask, nullptr, &usbServerTask_attributes);
}
