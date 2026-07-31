#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace ngc::ipc_detail {
    class SharedMemory {
    public:
        SharedMemory();
        ~SharedMemory();
        SharedMemory(SharedMemory &&) noexcept;
        SharedMemory &operator=(SharedMemory &&) noexcept;
        SharedMemory(const SharedMemory &) = delete;
        SharedMemory &operator=(const SharedMemory &) = delete;

        static SharedMemory create(std::string name, std::size_t size);
        static SharedMemory open(std::string name, std::size_t size);

        [[nodiscard]] void *data() const noexcept;
        [[nodiscard]] const std::string &name() const noexcept;
        void close() noexcept;

    private:
        class Impl;
        explicit SharedMemory(std::unique_ptr<Impl> impl);
        std::unique_ptr<Impl> m_impl;
    };

    class ChildProcess {
    public:
        ChildProcess();
        ~ChildProcess();
        ChildProcess(ChildProcess &&) noexcept;
        ChildProcess &operator=(ChildProcess &&) noexcept;
        ChildProcess(const ChildProcess &) = delete;
        ChildProcess &operator=(const ChildProcess &) = delete;

        static ChildProcess start(const std::filesystem::path &executable,
                                  const std::vector<std::string> &arguments);

        [[nodiscard]] bool running() const noexcept;
        [[nodiscard]] bool wait(std::chrono::milliseconds timeout) noexcept;
        void terminate() noexcept;
        void close() noexcept;

    private:
        class Impl;
        explicit ChildProcess(std::unique_ptr<Impl> impl);
        std::unique_ptr<Impl> m_impl;
    };

    [[nodiscard]] std::uint32_t currentProcessId() noexcept;
    [[nodiscard]] std::uint32_t parentProcessId() noexcept;
    [[nodiscard]] std::string uniqueSharedMemoryName();
}
