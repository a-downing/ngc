#include "IpcPlatform.h"

#include <atomic>
#include <format>
#include <stdexcept>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#else
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace ngc::ipc_detail {
    namespace {
        std::atomic<std::uint64_t> nextSharedMemoryId{1};

#ifdef _WIN32
        std::runtime_error windowsError(const std::string_view operation) {
            return std::runtime_error(std::format(
                "{} failed with Windows error {}", operation, GetLastError()));
        }

        std::wstring widen(const std::string_view value) {
            if (value.empty()) {
                return {};
            }

            const auto size = MultiByteToWideChar(
                CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                static_cast<int>(value.size()), nullptr, 0);
            if (size == 0) {
                throw windowsError("MultiByteToWideChar");
            }
            std::wstring result(static_cast<std::size_t>(size), L'\0');
            if (MultiByteToWideChar(
                    CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                    static_cast<int>(value.size()), result.data(), size) == 0) {
                throw windowsError("MultiByteToWideChar");
            }

            return result;
        }

        std::wstring quoteWindowsArgument(const std::wstring_view argument) {
            if (argument.find_first_of(L" \t\"") == std::wstring_view::npos) {
                return std::wstring(argument);
            }

            std::wstring result{L'"'};
            std::size_t backslashes = 0;
            for (const auto character : argument) {
                if (character == L'\\') {
                    ++backslashes;
                    continue;
                }
                if (character == L'"') {
                    result.append(backslashes * 2 + 1, L'\\');
                    result.push_back(L'"');
                    backslashes = 0;
                    continue;
                }
                result.append(backslashes, L'\\');
                backslashes = 0;
                result.push_back(character);
            }
            result.append(backslashes * 2, L'\\');
            result.push_back(L'"');

            return result;
        }
#else
        std::runtime_error posixError(const std::string_view operation) {
            return std::runtime_error(std::format(
                "{} failed: {}", operation, std::strerror(errno)));
        }
#endif
    }

    class SharedMemory::Impl {
    public:
        std::string name;
        void *data = nullptr;
        std::size_t size = 0;
        bool owner = false;
#ifdef _WIN32
        HANDLE mapping = nullptr;
#else
        int descriptor = -1;
#endif
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
#ifdef _WIN32
        const auto wideName = widen(impl->name);
        impl->mapping = CreateFileMappingW(
            INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
            static_cast<DWORD>(size >> 32), static_cast<DWORD>(size),
            wideName.c_str());
        if (impl->mapping == nullptr) {
            throw windowsError("CreateFileMappingW");
        }
        if (GetLastError() == ERROR_ALREADY_EXISTS) {
            CloseHandle(impl->mapping);
            throw std::runtime_error("shared-memory name already exists");
        }
        impl->data = MapViewOfFile(impl->mapping, FILE_MAP_ALL_ACCESS, 0, 0, size);
        if (impl->data == nullptr) {
            CloseHandle(impl->mapping);
            throw windowsError("MapViewOfFile");
        }
#else
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
#endif

        return SharedMemory(std::move(impl));
    }

    SharedMemory SharedMemory::open(std::string name, const std::size_t size) {
        auto impl = std::make_unique<Impl>();
        impl->name = std::move(name);
        impl->size = size;
#ifdef _WIN32
        const auto wideName = widen(impl->name);
        impl->mapping = OpenFileMappingW(
            FILE_MAP_ALL_ACCESS, FALSE, wideName.c_str());
        if (impl->mapping == nullptr) {
            throw windowsError("OpenFileMappingW");
        }
        impl->data = MapViewOfFile(impl->mapping, FILE_MAP_ALL_ACCESS, 0, 0, size);
        if (impl->data == nullptr) {
            CloseHandle(impl->mapping);
            throw windowsError("MapViewOfFile");
        }
#else
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
#endif

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
#ifdef _WIN32
        if (m_impl->data != nullptr) {
            UnmapViewOfFile(m_impl->data);
        }
        if (m_impl->mapping != nullptr) {
            CloseHandle(m_impl->mapping);
        }
#else
        if (m_impl->data != nullptr) {
            munmap(m_impl->data, m_impl->size);
        }
        if (m_impl->descriptor >= 0) {
            ::close(m_impl->descriptor);
        }
        if (m_impl->owner) {
            shm_unlink(m_impl->name.c_str());
        }
#endif
        m_impl.reset();
    }

    class ChildProcess::Impl {
    public:
#ifdef _WIN32
        HANDLE process = nullptr;
        HANDLE thread = nullptr;
#else
        pid_t process = -1;
        mutable bool reaped = false;
#endif
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
#ifdef _WIN32
        auto command = quoteWindowsArgument(executable.wstring());
        for (const auto &argument : arguments) {
            command.push_back(L' ');
            command += quoteWindowsArgument(widen(argument));
        }
        std::vector<wchar_t> mutableCommand(command.begin(), command.end());
        mutableCommand.push_back(L'\0');

        STARTUPINFOW startup{};
        startup.cb = sizeof(startup);
        PROCESS_INFORMATION process{};
        const auto success = CreateProcessW(
            executable.c_str(), mutableCommand.data(), nullptr, nullptr, FALSE,
            CREATE_NO_WINDOW, nullptr, nullptr, &startup, &process);
        if (!success) {
            throw windowsError("CreateProcessW");
        }
        impl->process = process.hProcess;
        impl->thread = process.hThread;
#else
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
#endif

        return ChildProcess(std::move(impl));
    }

    bool ChildProcess::running() const noexcept {
        if (!m_impl) {
            return false;
        }
#ifdef _WIN32
        return WaitForSingleObject(m_impl->process, 0) == WAIT_TIMEOUT;
#else
        if (m_impl->reaped) {
            return false;
        }
        int status = 0;
        const auto result = waitpid(m_impl->process, &status, WNOHANG);
        if (result == m_impl->process) {
            m_impl->reaped = true;
            return false;
        }

        return result == 0;
#endif
    }

    bool ChildProcess::wait(const std::chrono::milliseconds timeout) noexcept {
        if (!m_impl) {
            return true;
        }
#ifdef _WIN32
        return WaitForSingleObject(
            m_impl->process, static_cast<DWORD>(timeout.count())) == WAIT_OBJECT_0;
#else
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (!running()) {
                return true;
            }
            usleep(1000);
        }

        return !running();
#endif
    }

    void ChildProcess::terminate() noexcept {
        if (!m_impl || !running()) {
            return;
        }
#ifdef _WIN32
        TerminateProcess(m_impl->process, 1);
#else
        kill(m_impl->process, SIGTERM);
#endif
    }

    void ChildProcess::close() noexcept {
        if (!m_impl) {
            return;
        }
#ifdef _WIN32
        if (m_impl->thread != nullptr) {
            CloseHandle(m_impl->thread);
        }
        if (m_impl->process != nullptr) {
            CloseHandle(m_impl->process);
        }
#else
        if (!m_impl->reaped) {
            int status = 0;
            if (waitpid(m_impl->process, &status, WNOHANG) == m_impl->process) {
                m_impl->reaped = true;
            }
        }
#endif
        m_impl.reset();
    }

    std::uint32_t currentProcessId() noexcept {
#ifdef _WIN32
        return GetCurrentProcessId();
#else
        return static_cast<std::uint32_t>(getpid());
#endif
    }

    std::string uniqueSharedMemoryName() {
        const auto id = nextSharedMemoryId.fetch_add(1, std::memory_order_relaxed);
#ifdef _WIN32
        return std::format("Local\\ngc_ipc_{}_{}", currentProcessId(), id);
#else
        return std::format("/ngc_ipc_{}_{}", currentProcessId(), id);
#endif
    }
}
