#include <gtest/gtest.h>
#include "simuav/StatusServer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <string>

// ── helpers ──────────────────────────────────────────────────────────────────

// Creates a UDP socket bound to an ephemeral port and returns (fd, port).
static std::pair<int, uint16_t> makeReceiver() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return {-1, 0};

    // Bind to loopback on any free port.
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(0);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return {-1, 0};
    }

    // Query the assigned port.
    socklen_t len = sizeof(addr);
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);

    // Set a receive timeout so tests don't hang.
    timeval tv{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return {fd, ntohs(addr.sin_port)};
}

// ── tests ─────────────────────────────────────────────────────────────────────

TEST(StatusServer, PublishSendsValidJsonDatagram) {
    auto [rx_fd, port] = makeReceiver();
    ASSERT_GE(rx_fd, 0) << "Could not create receiver socket";

    simuav::StatusServer server(port);
    ASSERT_TRUE(server.isOpen());

    simuav::StatusSnapshot snap;
    snap.sim_time          = 1.234;
    snap.step_count        = 42;
    snap.overrun_count     = 2;
    snap.hil_sensor_sent   = 40;
    snap.actuator_received = 38;
    snap.baro_alt_m        = 488.5f;

    server.publish(snap);

    std::array<char, 1024> buf{};
    const ssize_t n = ::recv(rx_fd, buf.data(), buf.size() - 1, 0);
    ::close(rx_fd);

    ASSERT_GT(n, 0) << "No datagram received within 1 s";
    buf[n] = '\0';
    const std::string json(buf.data(), static_cast<std::size_t>(n));

    EXPECT_NE(json.find("sim_time"),          std::string::npos) << json;
    EXPECT_NE(json.find("step_count"),        std::string::npos) << json;
    EXPECT_NE(json.find("overrun_count"),     std::string::npos) << json;
    EXPECT_NE(json.find("hil_sensor_sent"),   std::string::npos) << json;
    EXPECT_NE(json.find("actuator_received"), std::string::npos) << json;
    EXPECT_NE(json.find("baro_alt_m"),        std::string::npos) << json;

    // Verify values are present in the output.
    EXPECT_NE(json.find("1.234"),  std::string::npos) << "sim_time value missing: " << json;
    EXPECT_NE(json.find("42"),     std::string::npos) << "step_count value missing: " << json;
    EXPECT_NE(json.find("488.5"),  std::string::npos) << "baro_alt_m value missing: " << json;
}

TEST(StatusServer, PublishIsNonBlocking) {
    // Even without a receiver the publish call must return immediately.
    // Use a port that no one is listening on.
    simuav::StatusServer server(59876);
    ASSERT_TRUE(server.isOpen());

    simuav::StatusSnapshot snap;
    snap.step_count = 1;

    // Just ensure it doesn't block or throw.
    for (int i = 0; i < 10; ++i) {
        server.publish(snap);
    }
    SUCCEED();
}
