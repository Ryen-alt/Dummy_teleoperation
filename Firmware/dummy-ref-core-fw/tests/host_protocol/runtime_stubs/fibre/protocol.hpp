#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
constexpr int CHANNEL_TYPE_USB = 1;
class PacketSink {
public:
    virtual ~PacketSink() = default;
    virtual int process_packet(const uint8_t*, size_t) = 0;
};
class StreamSink {
public:
    int channelType = 0;
    virtual ~StreamSink() = default;
    virtual int process_bytes(const uint8_t*, size_t, size_t*) = 0;
    virtual size_t get_free_space() = 0;
};
class BidirectionalPacketBasedChannel {
public:
    explicit BidirectionalPacketBasedChannel(PacketSink&) {}
    void process_packet(const uint8_t*, size_t) {}
};
