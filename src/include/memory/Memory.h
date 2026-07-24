#pragma once

#include <expected>

#include <algorithm>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <format>
#include <utility>
#include <stdexcept>
#include <span>

#include "memory/Vars.h"
#include "memory/MemoryCell.h"

namespace ngc
{
    class Memory {
        std::vector<MemoryCell> m_data;
        std::vector<double> m_stack;
        std::unordered_map<Var, uint32_t> m_globals;
        std::vector<uint32_t> m_addrs;
        std::size_t m_programStorageBegin = 0;

    public:
        struct PersistentParameter {
            uint32_t address;
            double value;

            auto operator<=>(const PersistentParameter &) const = default;
        };

        static constexpr uint32_t ADDR_STACK = 0x80000000;
        static constexpr uint32_t MAX_USER_PARAMETER = 5000;

        enum class Error {
            INVALID_DATA_ADDRESS,
            INVALID_STACK_ADDRESS,
            READ,
            WRITE
        };

        const std::vector<uint32_t> &addrs() const { return m_addrs; }

        void init(const std::span<const vars_t> specs) {
            m_data.clear();
            m_globals.clear();
            m_stack.clear();
            m_addrs.clear();

            for(const auto &[var, name, addr, flags, value] : specs) {
                while(m_data.size() < addr) {
                    if(m_data.size() <= MAX_USER_PARAMETER) {
                        addData(MemoryCell(MemoryCell::Flags::READ | MemoryCell::Flags::WRITE | MemoryCell::Flags::VOLATILE));
                    } else {
                        addData(MemoryCell(MemoryCell::Flags::NONE));
                    }
                }

                auto _addr = addData(MemoryCell(flags, value));

                m_globals.emplace(var, _addr);
                m_addrs.emplace_back(_addr);
            }

            m_programStorageBegin = m_data.size();
        }

        void resetProgramStorage() {
            m_data.erase(m_data.begin() + static_cast<std::ptrdiff_t>(m_programStorageBegin), m_data.end());
            m_stack.clear();
        }

        size_t deref(const Var var) const {
            if(!m_globals.contains(var)) {
                throw std::logic_error(std::format("Memory::read() unknown Var::{}", std::to_underlying(var)));
            }

            return m_globals.at(var);
        }

        double read(const Var var, const bool ignoreFlags = true) const {
            const auto addr = deref(var);
            const auto result = readData(addr, ignoreFlags);

            if(!result) {
                throw std::logic_error(std::format("Memory::readData failed for Var::{}", std::to_underlying(var)));
            }

            return *result;
        }

        void write(const Var var, const double value, const bool ignoreFlags = true) {
            if(!m_globals.contains(var)) {
                throw std::logic_error(std::format("Memory::write() unknown Var::{}", std::to_underlying(var)));
            }

            if(!writeData(m_globals.at(var), value, ignoreFlags)) {
                throw std::logic_error(std::format("Memory::writeData failed for Var::{}", std::to_underlying(var)));
            }
        }

        uint32_t addData(const MemoryCell mc) {
            const auto addr = m_data.size();
            m_data.emplace_back(mc);
            return addr;
        }

        uint32_t push(double value) {
            const auto addr = m_stack.size() | ADDR_STACK;
            m_stack.emplace_back(value);
            return addr;
        }

        double pop() {
            if(m_stack.empty()) {
                throw std::logic_error("Memory::pop called with an empty stack");
            }
            const double value = m_stack.back();
            m_stack.pop_back();
            return value;
        }

        std::expected<double, Error> read(const uint32_t addr, const bool ignoreFlags = false) const {
            if(addr == 0) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            if(addr & ADDR_STACK) {
                return readStack(addr & ~ADDR_STACK);
            }

            return readData(addr, ignoreFlags);
        }

        std::expected<void, Error> write(const uint32_t addr, const double value, const bool ignoreFlags = false) {
            if(addr == 0) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            if(addr & ADDR_STACK) {
                return writeStack(addr & ~ADDR_STACK, value);
            }

            return writeData(addr, value, ignoreFlags);
        }

        std::expected<bool, Error> isVolatile(const uint32_t addr) const {
            if(addr == 0) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            if(addr & ADDR_STACK) {
                return false;
            }

            return isVolatileData(addr);
        }

        [[nodiscard]] bool isPersistentParameter(const uint32_t addr) const {
            if (addr == 0 || (addr & ADDR_STACK) || addr >= m_data.size()) {
                return false;
            }

            return std::ranges::find(m_addrs, addr) != m_addrs.end()
                && !m_data[addr].volatileFlag();
        }

        [[nodiscard]] std::vector<PersistentParameter> persistentParameters() const {
            std::vector<PersistentParameter> result;
            result.reserve(m_addrs.size());
            for (const auto address : m_addrs) {
                if (isPersistentParameter(address)) {
                    result.push_back({address, m_data[address].read()});
                }
            }

            return result;
        }

        std::expected<void, Error> applyPersistentParameters(
            const std::span<const PersistentParameter> parameters) {
            for (const auto &[address, value] : parameters) {
                if (!isPersistentParameter(address)) {
                    return std::unexpected(Error::INVALID_DATA_ADDRESS);
                }
                if (!std::isfinite(value)) {
                    return std::unexpected(Error::WRITE);
                }
            }

            for (const auto &[address, value] : parameters) {
                m_data[address].write(value);
            }

            return {};
        }

    private:
        std::expected<double, Error> readData(const size_t index, const bool ignoreFlags) const {
            if(index >= m_data.size()) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            const auto &data = m_data.at(index);

            if(!data.readFlag() && !ignoreFlags) {
                return std::unexpected(Error::READ);
            }

            return data.read();
        }

        std::expected<void, Error> writeData(const size_t index, const double value, const bool ignoreFlags) {
            if(index >= m_data.size()) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            auto &data = m_data.at(index);

            if(!data.writeFlag() && !ignoreFlags) {
                return std::unexpected(Error::WRITE);
            }

            data.write(value);
            return {};
        }

        std::expected<bool, Error> isVolatileData(const size_t index) const {
            if(index >= m_data.size()) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            return m_data.at(index).volatileFlag();
        }

        std::expected<double, Error> readStack(const size_t index) const {
            if(index >= m_stack.size()) {
                return std::unexpected(Error::INVALID_STACK_ADDRESS);
            }

            return m_stack.at(index);
        }

        std::expected<void, Error> writeStack(const size_t index, const double value) {
            if(index >= m_stack.size()) {
                return std::unexpected(Error::INVALID_STACK_ADDRESS);
            }

            m_stack[index] = value;
            return {};
        }
    };
}
