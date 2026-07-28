#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <string>

#include "mesa/HostMot2Discovery.h"

namespace ngc::mesa {
    inline constexpr std::uint16_t LBP16_UDP_PORT = 27'181;
    inline constexpr std::size_t LBP16_MAX_DATAGRAM_SIZE = 1'400;

    struct Lbp16UdpConfiguration {
        std::string address;
        std::uint16_t port = LBP16_UDP_PORT;
        std::chrono::microseconds timeout =
            std::chrono::milliseconds(250);
    };

    enum class Lbp16DatagramStatus : std::uint8_t {
        Complete,
        InvalidRequest,
        SendFailed,
        PartialSend,
        ReceiveFailed,
        UnexpectedResponseSize,
    };

    struct Lbp16DatagramResult {
        Lbp16DatagramStatus status = Lbp16DatagramStatus::InvalidRequest;
        int systemError = 0;
        std::size_t sentBytes = 0;
        std::size_t receivedBytes = 0;
    };

    class Lbp16DatagramTransport {
    public:
        virtual ~Lbp16DatagramTransport() = default;

        [[nodiscard]] virtual Lbp16DatagramResult exchange(
            std::span<const std::byte> request,
            std::span<std::byte> response) noexcept = 0;
    };

    class Lbp16UdpTransport final : public HostMot2RegisterReader,
                                   public Lbp16DatagramTransport {
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
        [[nodiscard]] Lbp16DatagramResult exchange(
            std::span<const std::byte> request,
            std::span<std::byte> response) noexcept override;

    private:
        class Impl;

        explicit Lbp16UdpTransport(std::unique_ptr<Impl> impl);

        std::unique_ptr<Impl> m_impl;
    };
}
