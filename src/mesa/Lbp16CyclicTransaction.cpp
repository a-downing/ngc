#include "mesa/Lbp16CyclicTransaction.h"

#include <algorithm>
#include <format>
#include <string_view>

namespace ngc::mesa {
    namespace {
        constexpr std::size_t HOSTMOT2_WORD_SIZE = 4;
        constexpr std::size_t LBP16_MAX_ARGUMENTS = 0x7F;
        constexpr std::size_t SEQUENCE_SIZE = sizeof(std::uint32_t);
        constexpr std::size_t CONFIRMATION_SIZE = SEQUENCE_SIZE * 2;
        constexpr std::size_t BOARD_ERROR_SIZE = sizeof(std::uint16_t);
        constexpr std::size_t FINAL_REQUEST_SIZE =
            4 + CONFIRMATION_SIZE + 4 + 4;
        constexpr std::size_t FINAL_RESPONSE_SIZE =
            CONFIRMATION_SIZE + BOARD_ERROR_SIZE;
        constexpr std::uint32_t HOSTMOT2_ADDRESS_SPACE_SIZE = 0x1'0000;
        constexpr std::uint16_t LBP16_READ_WITH_ADDRESS = 0x4000;
        constexpr std::uint16_t LBP16_WRITE_WITH_ADDRESS = 0xC000;
        constexpr std::uint16_t LBP16_16_BIT_ARGUMENTS = 0x0100;
        constexpr std::uint16_t LBP16_32_BIT_ARGUMENTS = 0x0200;
        constexpr std::uint16_t LBP16_AUTO_INCREMENT = 0x0080;
        constexpr std::uint16_t LBP16_TIMER_SPACE = 0x1000;
        constexpr std::uint16_t LBP16_COMMUNICATION_SPACE = 0x1800;
        constexpr std::uint16_t LBP16_TIMER_SEQUENCE_ADDRESS = 0x0010;
        constexpr std::uint16_t LBP16_BOARD_ERROR_ADDRESS = 0x0000;

        std::uint32_t littleEndian32(
            const std::span<const std::byte> bytes) noexcept {
            return std::to_integer<std::uint32_t>(bytes[0])
                | (std::to_integer<std::uint32_t>(bytes[1]) << 8)
                | (std::to_integer<std::uint32_t>(bytes[2]) << 16)
                | (std::to_integer<std::uint32_t>(bytes[3]) << 24);
        }

        std::uint16_t littleEndian16(
            const std::span<const std::byte> bytes) noexcept {
            return static_cast<std::uint16_t>(
                std::to_integer<std::uint16_t>(bytes[0])
                | (std::to_integer<std::uint16_t>(bytes[1]) << 8));
        }

        void putLittleEndian32(
            const std::span<std::byte> bytes,
            const std::uint32_t value) noexcept {
            bytes[0] = static_cast<std::byte>(value);
            bytes[1] = static_cast<std::byte>(value >> 8);
            bytes[2] = static_cast<std::byte>(value >> 16);
            bytes[3] = static_cast<std::byte>(value >> 24);
        }
    }

    Lbp16CyclicTransaction::Lbp16CyclicTransaction(
        Lbp16DatagramTransport &transport) noexcept : m_transport(transport) { }

    std::expected<void, std::string>
    Lbp16CyclicTransaction::validateHostMot2Operation(
        const std::uint32_t address, const std::size_t size,
        const std::string_view description) const {
        if (m_finalized) {
            return std::unexpected(std::format(
                "cannot add an LBP16 HostMot2 {} after finalization",
                description));
        }
        if ((address & 0x3) != 0) {
            return std::unexpected(std::format(
                "LBP16 HostMot2 {} address 0x{:X} is not 32-bit aligned",
                description, address));
        }
        if (size == 0 || (size & 0x3) != 0) {
            return std::unexpected(std::format(
                "LBP16 HostMot2 {} size must be a positive multiple of four bytes",
                description));
        }
        if (size / HOSTMOT2_WORD_SIZE > LBP16_MAX_ARGUMENTS) {
            return std::unexpected(std::format(
                "LBP16 HostMot2 {} exceeds the {}-word command limit",
                description, LBP16_MAX_ARGUMENTS));
        }
        if (address >= HOSTMOT2_ADDRESS_SPACE_SIZE
            || size > HOSTMOT2_ADDRESS_SPACE_SIZE - address) {
            return std::unexpected(std::format(
                "LBP16 HostMot2 {} exceeds the 16-bit address space",
                description));
        }

        return {};
    }

    std::expected<void, std::string>
    Lbp16CyclicTransaction::appendCommand(
        const std::uint16_t command, const std::uint16_t address) {
        if (m_requestSize + 4 > m_request.size()) {
            return std::unexpected(
                "LBP16 cyclic request exceeds the datagram capacity");
        }

        m_request[m_requestSize] =
            static_cast<std::byte>(command);
        m_request[m_requestSize + 1] =
            static_cast<std::byte>(command >> 8);
        m_request[m_requestSize + 2] =
            static_cast<std::byte>(address);
        m_request[m_requestSize + 3] =
            static_cast<std::byte>(address >> 8);
        m_requestSize += 4;

        return {};
    }

    std::expected<Lbp16CyclicRead, std::string>
    Lbp16CyclicTransaction::addHostMot2Read(
        const std::uint32_t address, const std::size_t size) {
        if (const auto valid = validateHostMot2Operation(
                address, size, "read"); !valid) {
            return std::unexpected(valid.error());
        }
        if (m_readCount == m_reads.size()) {
            return std::unexpected(
                "LBP16 cyclic transaction has too many read operations");
        }
        if (m_requestSize + 4 + FINAL_REQUEST_SIZE > m_request.size()) {
            return std::unexpected(
                "LBP16 cyclic request exceeds the datagram capacity");
        }
        if (m_responseSize + size + FINAL_RESPONSE_SIZE > m_response.size()) {
            return std::unexpected(
                "LBP16 cyclic response exceeds the datagram capacity");
        }

        const auto words = static_cast<std::uint16_t>(
            size / HOSTMOT2_WORD_SIZE);
        const auto command = static_cast<std::uint16_t>(
            LBP16_READ_WITH_ADDRESS | LBP16_32_BIT_ARGUMENTS
            | LBP16_AUTO_INCREMENT | words);
        if (const auto appended = appendCommand(
                command, static_cast<std::uint16_t>(address)); !appended) {
            return std::unexpected(appended.error());
        }

        const auto index = m_readCount++;
        m_reads[index] = {
            .offset = m_responseSize,
            .size = size,
        };
        m_responseSize += size;

        return Lbp16CyclicRead{.index = index};
    }

    std::expected<Lbp16CyclicWrite, std::string>
    Lbp16CyclicTransaction::addHostMot2Write(
        const std::uint32_t address, const std::size_t size) {
        if (const auto valid = validateHostMot2Operation(
                address, size, "write"); !valid) {
            return std::unexpected(valid.error());
        }
        if (m_writeCount == m_writes.size()) {
            return std::unexpected(
                "LBP16 cyclic transaction has too many write operations");
        }
        if (m_requestSize + 4 + size + FINAL_REQUEST_SIZE
            > m_request.size()) {
            return std::unexpected(
                "LBP16 cyclic request exceeds the datagram capacity");
        }

        const auto words = static_cast<std::uint16_t>(
            size / HOSTMOT2_WORD_SIZE);
        const auto command = static_cast<std::uint16_t>(
            LBP16_WRITE_WITH_ADDRESS | LBP16_32_BIT_ARGUMENTS
            | LBP16_AUTO_INCREMENT | words);
        if (const auto appended = appendCommand(
                command, static_cast<std::uint16_t>(address)); !appended) {
            return std::unexpected(appended.error());
        }

        const auto index = m_writeCount++;
        m_writes[index] = {
            .offset = m_requestSize,
            .size = size,
        };
        m_requestSize += size;

        return Lbp16CyclicWrite{.index = index};
    }

    std::expected<void, std::string>
    Lbp16CyclicTransaction::finalize() {
        if (m_finalized) {
            return std::unexpected(
                "LBP16 cyclic transaction is already finalized");
        }
        if (m_requestSize + FINAL_REQUEST_SIZE > m_request.size()
            || m_responseSize + FINAL_RESPONSE_SIZE > m_response.size()) {
            return std::unexpected(
                "LBP16 cyclic transaction has no capacity for validation commands");
        }

        const auto writeSequenceCommand = static_cast<std::uint16_t>(
            LBP16_WRITE_WITH_ADDRESS | LBP16_TIMER_SPACE
            | LBP16_16_BIT_ARGUMENTS | LBP16_AUTO_INCREMENT | 4);
        if (const auto appended = appendCommand(
                writeSequenceCommand,
                LBP16_TIMER_SEQUENCE_ADDRESS); !appended) {
            return appended;
        }
        m_readSequenceRequestOffset = m_requestSize;
        m_writeSequenceRequestOffset =
            m_readSequenceRequestOffset + SEQUENCE_SIZE;
        m_requestSize += CONFIRMATION_SIZE;

        const auto readSequenceCommand = static_cast<std::uint16_t>(
            LBP16_READ_WITH_ADDRESS | LBP16_TIMER_SPACE
            | LBP16_16_BIT_ARGUMENTS | LBP16_AUTO_INCREMENT | 4);
        if (const auto appended = appendCommand(
                readSequenceCommand,
                LBP16_TIMER_SEQUENCE_ADDRESS); !appended) {
            return appended;
        }
        m_confirmationResponseOffset = m_responseSize;
        m_responseSize += CONFIRMATION_SIZE;

        const auto readErrorCommand = static_cast<std::uint16_t>(
            LBP16_READ_WITH_ADDRESS | LBP16_COMMUNICATION_SPACE
            | LBP16_16_BIT_ARGUMENTS | 1);
        if (const auto appended = appendCommand(
                readErrorCommand,
                LBP16_BOARD_ERROR_ADDRESS); !appended) {
            return appended;
        }
        m_boardErrorResponseOffset = m_responseSize;
        m_responseSize += BOARD_ERROR_SIZE;
        m_finalized = true;

        return {};
    }

    std::span<std::byte> Lbp16CyclicTransaction::writeData(
        const Lbp16CyclicWrite write) noexcept {
        if (write.index >= m_writeCount) {
            return {};
        }
        const auto &operation = m_writes[write.index];

        return std::span(m_request).subspan(
            operation.offset, operation.size);
    }

    std::span<const std::byte> Lbp16CyclicTransaction::readData(
        const Lbp16CyclicRead read) const noexcept {
        if (!m_hasValidInputs || read.index >= m_readCount) {
            return {};
        }
        const auto &operation = m_reads[read.index];

        return std::span(m_committedResponse).subspan(
            operation.offset, operation.size);
    }

    void Lbp16CyclicTransaction::invalidateInputs() noexcept {
        m_hasValidInputs = false;
        m_committedResponse.fill(std::byte{});
    }

    Lbp16CyclicResult Lbp16CyclicTransaction::exchange(
        const std::uint32_t readSequence,
        const std::uint32_t writeSequence) noexcept {
        auto result = Lbp16CyclicResult{};
        result.expectedReadSequence = readSequence;
        result.expectedWriteSequence = writeSequence;
        invalidateInputs();
        if (!m_finalized) {
            return result;
        }

        putLittleEndian32(
            std::span(m_request).subspan(
                m_readSequenceRequestOffset, SEQUENCE_SIZE),
            readSequence);
        putLittleEndian32(
            std::span(m_request).subspan(
                m_writeSequenceRequestOffset, SEQUENCE_SIZE),
            writeSequence);

        result.transport = m_transport.exchange(
            std::span(m_request).first(m_requestSize),
            std::span(m_response).first(m_responseSize));
        if (result.transport.status != Lbp16DatagramStatus::Complete
            || result.transport.receivedBytes != m_responseSize) {
            result.fault = Lbp16CyclicFault::Transport;

            return result;
        }

        const auto confirmation = std::span(m_response).subspan(
            m_confirmationResponseOffset, CONFIRMATION_SIZE);
        result.receivedReadSequence =
            littleEndian32(confirmation.first(SEQUENCE_SIZE));
        result.receivedWriteSequence =
            littleEndian32(confirmation.subspan(
                SEQUENCE_SIZE, SEQUENCE_SIZE));
        result.boardError = littleEndian16(
            std::span(m_response).subspan(
                m_boardErrorResponseOffset, BOARD_ERROR_SIZE));
        if (result.receivedReadSequence != readSequence) {
            result.fault = Lbp16CyclicFault::ReadSequenceMismatch;

            return result;
        }
        if (result.receivedWriteSequence != writeSequence) {
            result.fault = Lbp16CyclicFault::WriteSequenceMismatch;

            return result;
        }
        if (result.boardError != 0) {
            result.fault = Lbp16CyclicFault::BoardProtocolError;

            return result;
        }

        std::ranges::copy(
            std::span(m_response).first(m_responseSize),
            m_committedResponse.begin());
        m_hasValidInputs = true;
        result.fault = Lbp16CyclicFault::None;

        return result;
    }

    bool Lbp16CyclicTransaction::finalized() const noexcept {
        return m_finalized;
    }

    bool Lbp16CyclicTransaction::hasValidInputs() const noexcept {
        return m_hasValidInputs;
    }

    std::size_t Lbp16CyclicTransaction::requestSize() const noexcept {
        return m_requestSize;
    }

    std::size_t Lbp16CyclicTransaction::responseSize() const noexcept {
        return m_responseSize;
    }
}
