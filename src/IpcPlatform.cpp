#include "IpcPlatform.h"

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <format>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>

namespace ngc::ipc_detail {
    namespace {
        std::atomic<std::uint64_t> nextSharedMemoryId{1};

        std::runtime_error posixError(const std::string_view operation) {
            return std::runtime_error(std::format(
                "{} failed: {}", operation, std::strerror(errno)));
        }
    }

    class SharedMemory::Impl {
    public:
        std::string name;
        void *data = nullptr;
        std::size_t size = 0;
        bool owner = false;
        int descriptor = -1;
    };

    SharedMemory::SharedMemory() = default;

    SharedMemory::SharedMemory(std::unique_ptr<Impl> impl) : m_impl(std::move(impl)) { }

    SharedMemory::~SharedMemory() {
        close();
    }

    SharedMemory::SharedMemory(SharedMemory &&) noexcept = default;

    SharedMemory &SharedMemory::operator=(SharedMemory &&) noexcept = default;

    SharedMemory SharedMemory::create(std::string name, const std::size_t size) {
        auto impl = std::make_unique<Impl>();
        impl->name = std::move(name);
        impl->size = size;
        impl->owner = true;
        impl->descriptor = shm_open(
            impl->name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
        if (impl->descriptor < 0) {
            throw posixError("shm_open create");
        }
        if (ftruncate(impl->descriptor, static_cast<off_t>(size)) != 0) {
            const auto error = posixError("ftruncate");
            ::close(impl->descriptor);
            shm_unlink(impl->name.c_str());
            throw error;
        }
        impl->data = mmap(
            nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED,
            impl->descriptor, 0);
        if (impl->data == MAP_FAILED) {
            impl->data = nullptr;
            const auto error = posixError("mmap");
            ::close(impl->descriptor);
            shm_unlink(impl->name.c_str());
            throw error;
        }

        return SharedMemory(std::move(impl));
    }

    SharedMemory SharedMemory::open(std::string name, const std::size_t size) {
        auto impl = std::make_unique<Impl>();
        impl->name = std::move(name);
        impl->size = size;
        impl->descriptor = shm_open(impl->name.c_str(), O_RDWR, 0600);
        if (impl->descriptor < 0) {
            throw posixError("shm_open open");
        }
        impl->data = mmap(
            nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED,
            impl->descriptor, 0);
        if (impl->data == MAP_FAILED) {
            impl->data = nullptr;
            const auto error = posixError("mmap");
            ::close(impl->descriptor);
            throw error;
        }

        return SharedMemory(std::move(impl));
    }

    void *SharedMemory::data() const noexcept {
        return m_impl ? m_impl->data : nullptr;
    }

    const std::string &SharedMemory::name() const noexcept {
        static const std::string empty;

        return m_impl ? m_impl->name : empty;
    }

    void SharedMemory::close() noexcept {
        if (!m_impl) {
            return;
        }
        if (m_impl->data != nullptr) {
            munmap(m_impl->data, m_impl->size);
        }
        if (m_impl->descriptor >= 0) {
            ::close(m_impl->descriptor);
        }
        if (m_impl->owner) {
            shm_unlink(m_impl->name.c_str());
        }
        m_impl.reset();
    }

    class ChildProcess::Impl {
    public:
        pid_t process = -1;
        mutable bool reaped = false;
    };

    ChildProcess::ChildProcess() = default;

    ChildProcess::ChildProcess(std::unique_ptr<Impl> impl) : m_impl(std::move(impl)) { }

    ChildProcess::~ChildProcess() {
        close();
    }

    ChildProcess::ChildProcess(ChildProcess &&) noexcept = default;

    ChildProcess &ChildProcess::operator=(ChildProcess &&) noexcept = default;

    ChildProcess ChildProcess::start(
        const std::filesystem::path &executable,
        const std::vector<std::string> &arguments) {
        auto impl = std::make_unique<Impl>();
        const auto process = fork();
        if (process < 0) {
            throw posixError("fork");
        }
        if (process == 0) {
            std::vector<std::string> ownedArguments;
            ownedArguments.reserve(arguments.size() + 1);
            ownedArguments.push_back(executable.string());
            ownedArguments.insert(
                ownedArguments.end(), arguments.begin(), arguments.end());
            std::vector<char *> rawArguments;
            rawArguments.reserve(ownedArguments.size() + 1);
            for (auto &argument : ownedArguments) {
                rawArguments.push_back(argument.data());
            }
            rawArguments.push_back(nullptr);
            execv(executable.c_str(), rawArguments.data());
            _exit(127);
        }
        impl->process = process;

        return ChildProcess(std::move(impl));
    }

    bool ChildProcess::running() const noexcept {
        if (!m_impl || m_impl->reaped) {
            return false;
        }
        int status = 0;
        const auto result = waitpid(m_impl->process, &status, WNOHANG);
        if (result == m_impl->process) {
            m_impl->reaped = true;

            return false;
        }

        return result == 0;
    }

    bool ChildProcess::wait(const std::chrono::milliseconds timeout) noexcept {
        if (!m_impl) {
            return true;
        }
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (!running()) {
                return true;
            }
            usleep(1000);
        }

        return !running();
    }

    void ChildProcess::terminate() noexcept {
        if (!m_impl || !running()) {
            return;
        }

        kill(m_impl->process, SIGTERM);
    }

    void ChildProcess::close() noexcept {
        if (!m_impl) {
            return;
        }
        if (!m_impl->reaped) {
            int status = 0;
            if (waitpid(m_impl->process, &status, WNOHANG) == m_impl->process) {
                m_impl->reaped = true;
            }
        }
        m_impl.reset();
    }

    std::uint32_t currentProcessId() noexcept {
        return static_cast<std::uint32_t>(getpid());
    }

    std::string uniqueSharedMemoryName() {
        const auto id = nextSharedMemoryId.fetch_add(1, std::memory_order_relaxed);

        return std::format("/ngc_ipc_{}_{}", currentProcessId(), id);
    }
}
