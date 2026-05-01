#include "simuav/StatusServer.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <cstring>

namespace simuav {

StatusServer::StatusServer(uint16_t port) : port_(port) {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::perror("StatusServer: socket");
        return;
    }
    // Non-blocking: a full socket buffer just drops the datagram.
    const int flags = ::fcntl(sock_fd_, F_GETFL, 0);
    ::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);
}

StatusServer::~StatusServer() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void StatusServer::publish(const StatusSnapshot& s) {
    if (sock_fd_ < 0) return;

    std::array<char, 512> buf{};
    const int len = std::snprintf(buf.data(), buf.size(),
        "{\"sim_time\":%.6f,\"step_count\":%llu,\"overrun_count\":%llu,"
        "\"hil_sensor_sent\":%llu,\"actuator_received\":%llu,"
        "\"baro_alt_m\":%.3f}\n",
        s.sim_time,
        static_cast<unsigned long long>(s.step_count),
        static_cast<unsigned long long>(s.overrun_count),
        static_cast<unsigned long long>(s.hil_sensor_sent),
        static_cast<unsigned long long>(s.actuator_received),
        static_cast<double>(s.baro_alt_m));

    if (len <= 0) return;

    sockaddr_in dest{};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(port_);
    dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    ::sendto(sock_fd_, buf.data(), static_cast<std::size_t>(len), 0,
             reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
}

}  // namespace simuav
