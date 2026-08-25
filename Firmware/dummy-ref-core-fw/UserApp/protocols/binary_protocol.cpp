#include "binary_protocol.hpp"

#include <cstring>

namespace dummy::protocol
{
namespace
{
uint32_t ReadLittleEndianU32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

void WriteLittleEndianU32(uint32_t value, uint8_t* output)
{
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8U);
    output[2] = static_cast<uint8_t>(value >> 16U);
    output[3] = static_cast<uint8_t>(value >> 24U);
}
} // namespace

uint32_t Crc32c(const uint8_t* data, size_t length, uint32_t initial)
{
    uint32_t crc = initial ^ 0xFFFFFFFFU;
    for (size_t index = 0; index < length; ++index)
    {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0x82F63B78U & mask);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

size_t CobsEncode(const uint8_t* input, size_t input_length,
                  uint8_t* output, size_t output_capacity)
{
    if (output == nullptr || output_capacity == 0 || (input == nullptr && input_length != 0))
        return 0;

    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;
    if (write_index > output_capacity)
        return 0;

    while (read_index < input_length)
    {
        if (input[read_index] == 0)
        {
            if (code_index >= output_capacity)
                return 0;
            output[code_index] = code;
            code_index = write_index++;
            if (write_index > output_capacity)
                return 0;
            code = 1;
            ++read_index;
            continue;
        }

        if (write_index >= output_capacity)
            return 0;
        output[write_index++] = input[read_index++];
        ++code;
        if (code == 0xFF)
        {
            if (code_index >= output_capacity)
                return 0;
            output[code_index] = code;
            code_index = write_index++;
            if (write_index > output_capacity)
                return 0;
            code = 1;
        }
    }

    if (code_index >= output_capacity)
        return 0;
    output[code_index] = code;
    return write_index;
}

DecodeStatus CobsDecode(const uint8_t* input, size_t input_length,
                        uint8_t* output, size_t output_capacity,
                        size_t& output_length)
{
    output_length = 0;
    if (input == nullptr || input_length == 0)
        return DecodeStatus::Empty;
    if (output == nullptr)
        return DecodeStatus::Overflow;

    size_t read_index = 0;
    while (read_index < input_length)
    {
        const uint8_t code = input[read_index++];
        if (code == 0)
            return DecodeStatus::MalformedCobs;
        const size_t block_length = static_cast<size_t>(code) - 1;
        if (read_index + block_length > input_length)
            return DecodeStatus::MalformedCobs;
        if (output_length + block_length > output_capacity)
            return DecodeStatus::Overflow;
        std::memcpy(output + output_length, input + read_index, block_length);
        output_length += block_length;
        read_index += block_length;
        if (code != 0xFF && read_index < input_length)
        {
            if (output_length >= output_capacity)
                return DecodeStatus::Overflow;
            output[output_length++] = 0;
        }
    }
    return DecodeStatus::Ok;
}

size_t EncodePacket(const Packet& packet, uint8_t* output, size_t output_capacity)
{
    if (packet.header.payload_length > kMaxPayload || output_capacity < 2)
        return 0;

    std::array<uint8_t, kMaxDecodedFrame> decoded{};
    PacketHeader header = packet.header;
    header.magic = kMagic;
    header.version = kProtocolVersion;
    std::memcpy(decoded.data(), &header, sizeof(header));
    std::memcpy(decoded.data() + sizeof(header), packet.payload.data(), header.payload_length);
    const size_t body_length = sizeof(header) + header.payload_length;
    WriteLittleEndianU32(Crc32c(decoded.data(), body_length), decoded.data() + body_length);
    const size_t encoded_length = CobsEncode(decoded.data(), body_length + kCrcSize,
                                             output, output_capacity - 1);
    if (encoded_length == 0 || encoded_length >= output_capacity)
        return 0;
    output[encoded_length] = 0;
    return encoded_length + 1;
}

DecodeStatus DecodePacket(const uint8_t* encoded, size_t encoded_length, Packet& packet)
{
    if (encoded_length > 0 && encoded[encoded_length - 1] == 0)
        --encoded_length;
    std::array<uint8_t, kMaxDecodedFrame> decoded{};
    size_t decoded_length = 0;
    const DecodeStatus cobs_status = CobsDecode(encoded, encoded_length, decoded.data(),
                                                decoded.size(), decoded_length);
    if (cobs_status != DecodeStatus::Ok)
        return cobs_status;
    if (decoded_length < sizeof(PacketHeader) + kCrcSize)
        return DecodeStatus::TooShort;

    const size_t body_length = decoded_length - kCrcSize;
    const uint32_t expected_crc = ReadLittleEndianU32(decoded.data() + body_length);
    if (Crc32c(decoded.data(), body_length) != expected_crc)
        return DecodeStatus::BadCrc;

    std::memcpy(&packet.header, decoded.data(), sizeof(packet.header));
    if (packet.header.magic != kMagic)
        return DecodeStatus::BadMagic;
    if (packet.header.version != kProtocolVersion)
        return DecodeStatus::BadVersion;
    if (!IsKnownMessage(packet.header.message_type))
        return DecodeStatus::UnknownMessage;
    if (packet.header.payload_length > kMaxPayload ||
        sizeof(PacketHeader) + packet.header.payload_length != body_length)
        return DecodeStatus::BadLength;
    std::memcpy(packet.payload.data(), decoded.data() + sizeof(PacketHeader),
                packet.header.payload_length);
    return DecodeStatus::Ok;
}

bool IsKnownMessage(uint8_t raw_type)
{
    switch (static_cast<MessageType>(raw_type))
    {
        case MessageType::Hello:
        case MessageType::AcquireControl:
        case MessageType::ReleaseControl:
        case MessageType::SetMode:
        case MessageType::Heartbeat:
        case MessageType::SetJointTarget:
        case MessageType::Hold:
        case MessageType::EmergencyStop:
        case MessageType::ClearFault:
        case MessageType::TargetKeepalive:
        case MessageType::TimeSync:
        case MessageType::GetCanDiagnostics:
        case MessageType::HelloAck:
        case MessageType::State:
        case MessageType::Ack:
        case MessageType::Nack:
        case MessageType::Fault:
        case MessageType::Event:
        case MessageType::TimeSyncAck:
        case MessageType::CanDiagnostics:
            return true;
    }
    return false;
}

bool StreamDecoder::Feed(uint8_t byte, Packet& packet)
{
    if (byte != 0)
    {
        if (length_ >= buffer_.size())
        {
            length_ = 0;
            ++dropped_frames_;
            last_error_ = DecodeStatus::Overflow;
        }
        else
        {
            buffer_[length_++] = byte;
        }
        return false;
    }
    if (length_ == 0)
        return false;

    last_error_ = DecodePacket(buffer_.data(), length_, packet);
    length_ = 0;
    if (last_error_ != DecodeStatus::Ok)
    {
        ++dropped_frames_;
        return false;
    }
    return true;
}

} // namespace dummy::protocol
