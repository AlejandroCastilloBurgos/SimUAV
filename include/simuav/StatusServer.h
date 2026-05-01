#pragma once
#include <cstdint>

namespace simuav {

struct StatusSnapshot {
    double   sim_time{0.0};
    uint64_t step_count{0};
    uint64_t overrun_count{0};
    uint64_t hil_sensor_sent{0};
    uint64_t actuator_received{0};
    float    baro_alt_m{0.0f};
};

// Publishes StatusSnapshot as a JSON datagram to localhost:port.
// Non-blocking and best-effort — a full socket buffer is silently dropped.
class StatusServer {
public:
    explicit StatusServer(uint16_t port);
    ~StatusServer();

    // Returns true if the socket opened successfully.
    bool isOpen() const { return sock_fd_ >= 0; }
    uint16_t port() const { return port_; }

    void publish(const StatusSnapshot& snap);

private:
    int      sock_fd_{-1};
    uint16_t port_{0};
};

}  // namespace simuav
