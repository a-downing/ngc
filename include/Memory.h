#ifndef MEMORY_H
#define MEMORY_H

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <format>
#include <utility>
#include <stdexcept>

#include <Vars.h>

namespace ngc
{
    class Memory {
        std::vector<MemoryCell> m_data;
        std::vector<double> m_stack;
        std::unordered_map<Var, uint32_t> m_globals;

    public:
        static constexpr uint32_t ADDR_STACK = 0x80000000;

        enum class Error {
            INVALID_DATA_ADDRESS,
            INVALID_STACK_ADDRESS,
            READ,
            WRITE
        };

        std::vector<uint32_t> init(std::initializer_list<std::tuple<Var, std::string_view, MemoryCell::Flags>> specs) {
            m_data.clear();
            m_globals.clear();
            m_stack.clear();

            std::vector<uint32_t> addrs;

            for(const auto &[var, name, flags] : specs) {
                const auto addr = addData(MemoryCell(flags));
                m_globals.emplace(var, addr);
                addrs.emplace_back(addr);
            }

            return addrs;
        }

        double read(const Var var) const {
            if(!m_globals.contains(var)) {
                throw std::logic_error(std::format("Memory::read() unknown Var::{}", std::to_underlying(var)));
            }

            const auto result = readData(m_globals.at(var), true);

            if(!result) {
                throw std::logic_error(std::format("Memory::readData failed for Var::{}", std::to_underlying(var)));
            }

            return *result;
        }

        void write(const Var var, const double value) {
            if(!m_globals.contains(var)) {
                throw std::logic_error(std::format("Memory::write() unknown Var::{}", std::to_underlying(var)));
            }

            if(!writeData(m_globals.at(var), value, true)) {
                throw std::logic_error(std::format("Memory::writeData failed for Var::{}", std::to_underlying(var)));
            }
        }

        uint32_t addData(const MemoryCell mc) {
            m_data.emplace_back(mc);
            return m_data.size();
        }

        uint32_t push(double value) {
            m_stack.emplace_back(value);
            return m_stack.size() | ADDR_STACK;
        }

        double pop() {
            const double value = m_stack.back();
            m_stack.pop_back();
            return value;
        }

        std::expected<double, Error> read(const uint32_t addr) const {
            if(addr == 0) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            if(addr & ADDR_STACK) {
                return readStack((addr & ~ADDR_STACK) - 1);
            }

            return readData(addr - 1, false);
        }

        std::expected<void, Error> write(const uint32_t addr, const double value) {
            if(addr == 0) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            if(addr & ADDR_STACK) {
                return writeStack((addr & ~ADDR_STACK) - 1, value);
            }

            return writeData(addr - 1, value, false);
        }

        std::expected<bool, Error> isVolatile(const uint32_t addr) const {
            if(addr == 0) {
                return std::unexpected(Error::INVALID_DATA_ADDRESS);
            }

            if(addr & ADDR_STACK) {
                return isVolatileStack((addr & ~ADDR_STACK) - 1);
            }

            return isVolatileData(addr - 1);
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
            if(index >= m_data.size()) {
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

        std::expected<bool, Error> isVolatileStack(const size_t index) const {
            if(index >= m_stack.size()) {
                return std::unexpected(Error::INVALID_STACK_ADDRESS);
            }

            return false;
        }
    };
}

#endif //MEMORY_H
