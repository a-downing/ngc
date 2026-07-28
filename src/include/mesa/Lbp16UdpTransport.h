#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>

#include "mesa/HostMot2Discovery.h"

namespace ngc::mesa {
    inline constexpr std::uint16_t LBP16_UDP_PORT = 27'181;

    struct Lbp16UdpConfiguration {
        std::string address;
        std::uint16_t port = LBP16_UDP_PORT;
        std::chrono::milliseconds timeout{250};
    };

    class Lbp16UdpTransport final : public HostMot2RegisterReader {
    public:
        [[nodiscard]] static std::expected<
            std::unique_ptr<Lbp16UdpTransport>, std::string>
        open(const Lbp16UdpConfiguration &configuration);

        ~Lbp16UdpTransport() override;
        Lbp16UdpTransport(const Lbp16UdpTransport &) = delete;
        Lbp16UdpTransport &operator=(const Lbp16UdpTransport &) = delete;

        [[nodiscard]] std::expected<void, std::string> read(
            std::uint32_t address,
            std::span<std::byte> destination) override;

    private:
        class Impl;

        explicit Lbp16UdpTransport(std::unique_ptr<Impl> impl);

        std::unique_ptr<Impl> m_impl;
    };
}
