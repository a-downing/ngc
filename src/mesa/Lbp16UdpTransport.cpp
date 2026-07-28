#include "mesa/Lbp16UdpTransport.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <format>
#include <limits>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <WinSock2.h>
#include <WS2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace ngc::mesa {
    namespace {
        constexpr std::size_t LBP16_MAX_WORDS = 0x7F;
        constexpr std::size_t HOSTMOT2_WORD_SIZE = 4;
        constexpr std::size_t LBP16_MAX_READ_SIZE =
            LBP16_MAX_WORDS * HOSTMOT2_WORD_SIZE;
        constexpr std::uint16_t LBP16_READ_WITH_ADDRESS = 0x4000;
        constexpr std::uint16_t LBP16_32_BIT_ARGUMENTS = 0x0200;
        constexpr std::uint16_t LBP16_AUTO_INCREMENT = 0x0080;
        constexpr std::uint32_t LBP16_ADDRESS_SPACE_SIZE = 0x1'0000;

        std::array<std::byte, 4> readRequest(
            const std::uint16_t address,
            const std::uint8_t words) noexcept {
            const auto command = static_cast<std::uint16_t>(
                LBP16_READ_WITH_ADDRESS | LBP16_32_BIT_ARGUMENTS
                | LBP16_AUTO_INCREMENT | words);

            return {
                static_cast<std::byte>(command & 0xFF),
                static_cast<std::byte>(command >> 8),
                static_cast<std::byte>(address & 0xFF),
                static_cast<std::byte>(address >> 8),
            };
        }

#ifdef _WIN32
        std::string socketError(
            const std::string_view operation) {
            return std::format(
                "{} failed with Windows socket error {}",
                operation, WSAGetLastError());
        }

        int lastSocketError() noexcept {
            return WSAGetLastError();
        }
#else
        std::string socketError(
            const std::string_view operation) {
            return std::format(
                "{} failed: {}", operation, std::strerror(errno));
        }

        int lastSocketError() noexcept {
            return errno;
        }
#endif
    }

    class Lbp16UdpTransport::Impl {
    public:
#ifdef _WIN32
        using Socket = SOCKET;
        static constexpr Socket INVALID_SOCKET_VALUE = INVALID_SOCKET;
#else
        using Socket = int;
        static constexpr Socket INVALID_SOCKET_VALUE = -1;
#endif

        Socket socket = INVALID_SOCKET_VALUE;
        std::string address;

#ifdef _WIN32
        bool winsockStarted = false;
#endif

        ~Impl() {
            close();
        }

        void close() noexcept {
            if (socket != INVALID_SOCKET_VALUE) {
#ifdef _WIN32
                closesocket(socket);
#else
                ::close(socket);
#endif
                socket = INVALID_SOCKET_VALUE;
            }
#ifdef _WIN32
            if (winsockStarted) {
                WSACleanup();
                winsockStarted = false;
            }
#endif
        }

        std::expected<void, std::string> readExchange(
            const std::uint16_t addressValue,
            const std::span<std::byte> destination) {
            const auto words = static_cast<std::uint8_t>(
                destination.size() / HOSTMOT2_WORD_SIZE);
            const auto request = readRequest(addressValue, words);
#ifdef _WIN32
            const auto sent = ::send(
                socket, reinterpret_cast<const char *>(request.data()),
                static_cast<int>(request.size()), 0);
#else
            const auto sent = ::send(
                socket, request.data(), request.size(), 0);
#endif
            if (sent < 0) {
                return std::unexpected(socketError("LBP16 UDP send"));
            }
            if (static_cast<std::size_t>(sent) != request.size()) {
                return std::unexpected(std::format(
                    "LBP16 UDP send transmitted {} of {} request bytes",
                    sent, request.size()));
            }

#ifdef _WIN32
            const auto received = ::recv(
                socket, reinterpret_cast<char *>(destination.data()),
                static_cast<int>(destination.size()), 0);
#else
            const auto received = ::recv(
                socket, destination.data(), destination.size(), 0);
#endif
            if (received < 0) {
                return std::unexpected(socketError("LBP16 UDP receive"));
            }
            if (static_cast<std::size_t>(received) != destination.size()) {
                return std::unexpected(std::format(
                    "LBP16 UDP response from {} contained {} bytes; expected {}",
                    address, received, destination.size()));
            }

            return {};
        }

        Lbp16DatagramResult exchangeDatagram(
            const std::span<const std::byte> request,
            const std::span<std::byte> response) noexcept {
            auto result = Lbp16DatagramResult{};
            if (request.empty() || response.empty()
                || request.size() > LBP16_MAX_DATAGRAM_SIZE
                || response.size() > LBP16_MAX_DATAGRAM_SIZE) {
                return result;
            }

#ifdef _WIN32
            const auto sent = ::send(
                socket, reinterpret_cast<const char *>(request.data()),
                static_cast<int>(request.size()), 0);
#else
            const auto sent = ::send(
                socket, request.data(), request.size(), 0);
#endif
            if (sent < 0) {
                result.status = Lbp16DatagramStatus::SendFailed;
                result.systemError = lastSocketError();

                return result;
            }
            result.sentBytes = static_cast<std::size_t>(sent);
            if (result.sentBytes != request.size()) {
                result.status = Lbp16DatagramStatus::PartialSend;

                return result;
            }

            std::array<
                std::byte,
                LBP16_MAX_DATAGRAM_SIZE + 1> datagram{};
#ifdef _WIN32
            const auto received = ::recv(
                socket, reinterpret_cast<char *>(datagram.data()),
                static_cast<int>(response.size() + 1), 0);
#else
            const auto received = ::recv(
                socket, datagram.data(), response.size() + 1, 0);
#endif
            if (received < 0) {
                result.status = Lbp16DatagramStatus::ReceiveFailed;
                result.systemError = lastSocketError();

                return result;
            }
            result.receivedBytes = static_cast<std::size_t>(received);
            if (result.receivedBytes != response.size()) {
                result.status =
                    Lbp16DatagramStatus::UnexpectedResponseSize;

                return result;
            }

            std::ranges::copy(
                std::span(datagram).first(response.size()),
                response.begin());
            result.status = Lbp16DatagramStatus::Complete;

            return result;
        }
    };

    std::expected<std::unique_ptr<Lbp16UdpTransport>, std::string>
    Lbp16UdpTransport::open(
        const Lbp16UdpConfiguration &configuration) {
        if (configuration.address.empty()) {
            return std::unexpected(
                "LBP16 UDP address must not be empty");
        }
        if (configuration.timeout <= std::chrono::milliseconds::zero()) {
            return std::unexpected(
                "LBP16 UDP timeout must be positive");
        }

        auto impl = std::make_unique<Impl>();
        impl->address = configuration.address;
#ifdef _WIN32
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            return std::unexpected(socketError("WSAStartup"));
        }
        impl->winsockStarted = true;
#endif
        impl->socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (impl->socket == Impl::INVALID_SOCKET_VALUE) {
            return std::unexpected(socketError("LBP16 UDP socket"));
        }

        sockaddr_in endpoint{};
        endpoint.sin_family = AF_INET;
        endpoint.sin_port = htons(configuration.port);
        if (inet_pton(
                AF_INET, configuration.address.c_str(),
                &endpoint.sin_addr) != 1) {
            return std::unexpected(std::format(
                "LBP16 UDP address '{}' is not a valid IPv4 address",
                configuration.address));
        }
        if (::connect(
                impl->socket,
                reinterpret_cast<const sockaddr *>(&endpoint),
                sizeof(endpoint)) != 0) {
            return std::unexpected(socketError("LBP16 UDP connect"));
        }

#ifdef _WIN32
        const auto timeoutMilliseconds =
            std::chrono::ceil<std::chrono::milliseconds>(
                configuration.timeout);
        const auto timeout = static_cast<DWORD>(
            std::min<std::int64_t>(
                timeoutMilliseconds.count(),
                std::numeric_limits<DWORD>::max()));
        if (setsockopt(
                impl->socket, SOL_SOCKET, SO_RCVTIMEO,
                reinterpret_cast<const char *>(&timeout),
                sizeof(timeout)) != 0
            || setsockopt(
                impl->socket, SOL_SOCKET, SO_SNDTIMEO,
                reinterpret_cast<const char *>(&timeout),
                sizeof(timeout)) != 0) {
            return std::unexpected(
                socketError("LBP16 UDP timeout configuration"));
        }
#else
        const auto seconds =
            std::chrono::duration_cast<std::chrono::seconds>(
                configuration.timeout);
        const auto remainder = configuration.timeout - seconds;
        const timeval timeout{
            .tv_sec = static_cast<time_t>(seconds.count()),
            .tv_usec = static_cast<suseconds_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    remainder).count()),
        };
        if (setsockopt(
                impl->socket, SOL_SOCKET, SO_RCVTIMEO,
                &timeout, sizeof(timeout)) != 0
            || setsockopt(
                impl->socket, SOL_SOCKET, SO_SNDTIMEO,
                &timeout, sizeof(timeout)) != 0) {
            return std::unexpected(
                socketError("LBP16 UDP timeout configuration"));
        }
#endif

        return std::unique_ptr<Lbp16UdpTransport>(
            new Lbp16UdpTransport(std::move(impl)));
    }

    Lbp16UdpTransport::Lbp16UdpTransport(
        std::unique_ptr<Impl> impl) : m_impl(std::move(impl)) { }

    Lbp16UdpTransport::~Lbp16UdpTransport() = default;

    std::expected<void, std::string> Lbp16UdpTransport::read(
        const std::uint32_t address,
        const std::span<std::byte> destination) {
        if ((address & 0x3) != 0) {
            return std::unexpected(std::format(
                "LBP16 HostMot2 address 0x{:X} is not 32-bit aligned",
                address));
        }
        if ((destination.size() & 0x3) != 0) {
            return std::unexpected(
                "LBP16 HostMot2 read size is not a multiple of four bytes");
        }
        if (address >= LBP16_ADDRESS_SPACE_SIZE
            || destination.size() > LBP16_ADDRESS_SPACE_SIZE - address) {
            return std::unexpected(
                "LBP16 HostMot2 read exceeds the 16-bit address space");
        }

        auto remaining = destination;
        auto currentAddress = address;
        while (!remaining.empty()) {
            const auto size =
                std::min(remaining.size(), LBP16_MAX_READ_SIZE);
            const auto chunk = remaining.first(size);
            const auto result = m_impl->readExchange(
                static_cast<std::uint16_t>(currentAddress), chunk);
            if (!result) {
                return result;
            }
            remaining = remaining.subspan(size);
            currentAddress += static_cast<std::uint32_t>(size);
        }

        return {};
    }

    Lbp16DatagramResult Lbp16UdpTransport::exchange(
        const std::span<const std::byte> request,
        const std::span<std::byte> response) noexcept {
        return m_impl->exchangeDatagram(request, response);
    }
}
