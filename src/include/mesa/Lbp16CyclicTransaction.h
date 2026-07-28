#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>

#include "mesa/Lbp16UdpTransport.h"

namespace ngc::mesa {
    struct Lbp16CyclicRead {
        std::size_t index = 0;
    };

    struct Lbp16CyclicWrite {
        std::size_t index = 0;
    };

    enum class Lbp16CyclicFault : std::uint8_t {
        None,
        NotFinalized,
        Transport,
        ReadSequenceMismatch,
        WriteSequenceMismatch,
        BoardProtocolError,
    };

    struct Lbp16CyclicResult {
        Lbp16CyclicFault fault = Lbp16CyclicFault::NotFinalized;
        Lbp16DatagramResult transport;
        std::uint32_t expectedReadSequence = 0;
        std::uint32_t receivedReadSequence = 0;
        std::uint32_t expectedWriteSequence = 0;
        std::uint32_t receivedWriteSequence = 0;
        std::uint16_t boardError = 0;
    };

    class Lbp16CyclicTransaction {
    public:
        static constexpr std::size_t MAX_OPERATIONS = 64;

        explicit Lbp16CyclicTransaction(
            Lbp16DatagramTransport &transport) noexcept;

        [[nodiscard]] std::expected<Lbp16CyclicRead, std::string>
        addHostMot2Read(std::uint32_t address, std::size_t size);
        [[nodiscard]] std::expected<Lbp16CyclicWrite, std::string>
        addHostMot2Write(std::uint32_t address, std::size_t size);
        [[nodiscard]] std::expected<void, std::string> finalize();

        [[nodiscard]] std::span<std::byte> writeData(
            Lbp16CyclicWrite write) noexcept;
        [[nodiscard]] std::span<const std::byte> readData(
            Lbp16CyclicRead read) const noexcept;
        [[nodiscard]] Lbp16CyclicResult exchange(
            std::uint32_t readSequence,
            std::uint32_t writeSequence) noexcept;

        [[nodiscard]] bool finalized() const noexcept;
        [[nodiscard]] bool hasValidInputs() const noexcept;
        [[nodiscard]] std::size_t requestSize() const noexcept;
        [[nodiscard]] std::size_t responseSize() const noexcept;

    private:
        struct Operation {
            std::size_t offset = 0;
            std::size_t size = 0;
        };

        [[nodiscard]] std::expected<void, std::string>
        validateHostMot2Operation(
            std::uint32_t address, std::size_t size,
            std::string_view description) const;
        [[nodiscard]] std::expected<void, std::string>
        appendCommand(
            std::uint16_t command, std::uint16_t address);
        void invalidateInputs() noexcept;

        Lbp16DatagramTransport &m_transport;
        std::array<std::byte, LBP16_MAX_DATAGRAM_SIZE> m_request{};
        std::array<std::byte, LBP16_MAX_DATAGRAM_SIZE> m_response{};
        std::array<std::byte, LBP16_MAX_DATAGRAM_SIZE> m_committedResponse{};
        std::array<Operation, MAX_OPERATIONS> m_reads{};
        std::array<Operation, MAX_OPERATIONS> m_writes{};
        std::size_t m_requestSize = 0;
        std::size_t m_responseSize = 0;
        std::size_t m_readCount = 0;
        std::size_t m_writeCount = 0;
        std::size_t m_readSequenceRequestOffset = 0;
        std::size_t m_writeSequenceRequestOffset = 0;
        std::size_t m_confirmationResponseOffset = 0;
        std::size_t m_boardErrorResponseOffset = 0;
        bool m_finalized = false;
        bool m_hasValidInputs = false;
    };
}
