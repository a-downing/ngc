#pragma once

#include <expected>

#include <format>
#include <print>
#include <string>
#include <filesystem>
#include <fstream>
#include <charconv>
#include <array>
#include <source_location>
#include <limits>
#include <atomic>

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#endif

#define PANIC(...) ngc::panic(std::source_location::current(), "PANIC" __VA_OPT__(,) __VA_ARGS__)
#define UNREACHABLE(...) ngc::panic(std::source_location::current(), "UNREACHABLE" __VA_OPT__(,) __VA_ARGS__)

namespace ngc
{
    template<typename ...Args>
    [[noreturn]] void panic(const std::source_location loc, const std::string_view tag, std::format_string<Args...> fmt, Args&&... args) {
        std::println(stderr, "{}:{}:{}: {}: {}: {}", loc.file_name(), loc.line(), loc.column(), loc.function_name(), tag, std::format(fmt, std::forward<Args>(args)...));
        std::exit(1);
    }

    [[noreturn]] inline void panic(const std::source_location loc, const std::string_view tag) {
        std::println(stderr, "{}:{}:{}: {}: {}", loc.file_name(), loc.line(), loc.column(), loc.function_name(), tag);
        std::exit(1);
    }

    [[noreturn]] inline void panic(const std::source_location loc, const std::string_view tag, std::string_view text) {
        std::println(stderr, "{}:{}:{}: {}: {}: {}", loc.file_name(), loc.line(), loc.column(), loc.function_name(), tag, text);
        std::exit(1);
    }

    inline std::string toChars(const double d) {
        std::array<char, 32> chars;
        auto [ptr, ec] = std::to_chars(chars.data(), chars.data() + chars.size(), d);

        if(ec != std::errc()) {
            throw std::runtime_error(std::format("{}() failed to convert {}", __func__, d));
        }

        return std::string(chars.data(), ptr);
    }

    inline std::expected<double, std::errc> fromChars(const std::string_view text) {
        double d;
        auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), d);

        if(ec != std::errc()) {
            return std::unexpected(ec);
        }

        if(ptr != text.data() + text.size()) {
            return std::unexpected(std::errc::invalid_argument);
        }

        return d;
    }

    inline std::expected<std::string, std::ios_base::failure> readFile(const std::filesystem::path& filePath) {
        std::ifstream file(filePath, std::ios::binary);

        if (!file) {
            return std::unexpected(std::ios_base::failure(std::format("failed to open '{}'", filePath.string())));
        }

        file.seekg(0, std::ios::end);
        const auto fileSize = file.tellg();
        if(fileSize < 0) {
            return std::unexpected(std::ios_base::failure(std::format("failed to determine size of '{}'", filePath.string())));
        }
        if(static_cast<std::uintmax_t>(fileSize) > std::numeric_limits<std::size_t>::max()) {
            return std::unexpected(std::ios_base::failure(std::format("'{}' is too large to read", filePath.string())));
        }
        file.seekg(0, std::ios::beg);
        if(!file) {
            return std::unexpected(std::ios_base::failure(std::format("failed to seek '{}'", filePath.string())));
        }

        std::string fileContent(static_cast<std::size_t>(fileSize), '\0');
        if(!fileContent.empty()) {
            file.read(fileContent.data(), static_cast<std::streamsize>(fileContent.size()));
            if(!file || file.gcount() != static_cast<std::streamsize>(fileContent.size())) {
                return std::unexpected(std::ios_base::failure(std::format("failed to read all of '{}'", filePath.string())));
            }
        }

        return fileContent;
    }

    inline std::expected<void, std::ios_base::failure> writeFile(const std::filesystem::path& filePath, const std::string_view text) {
        static std::atomic_uint64_t nextTemporaryId = 1;
        auto temporaryPath = filePath;
        temporaryPath += std::format(".tmp.{}", nextTemporaryId.fetch_add(1, std::memory_order_relaxed));

        std::ofstream file(temporaryPath, std::ios::binary | std::ios::trunc);

        if (!file) {
            return std::unexpected(std::ios_base::failure(std::format("failed to open temporary file '{}'", temporaryPath.string())));
        }

        file.write(text.data(), static_cast<std::streamsize>(text.size()));
        file.flush();
        if(!file) {
            file.close();
            std::error_code ignored;
            std::filesystem::remove(temporaryPath, ignored);
            return std::unexpected(std::ios_base::failure(std::format("failed to write all of '{}'", filePath.string())));
        }

        file.close();
        if(!file) {
            std::error_code ignored;
            std::filesystem::remove(temporaryPath, ignored);
            return std::unexpected(std::ios_base::failure(std::format("failed to close temporary file for '{}'", filePath.string())));
        }

#ifdef _WIN32
        if(!MoveFileExW(temporaryPath.c_str(), filePath.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
            const auto error = GetLastError();
            std::error_code ignored;
            std::filesystem::remove(temporaryPath, ignored);
            return std::unexpected(std::ios_base::failure(
                std::format("failed to replace '{}' (Windows error {})", filePath.string(), error)));
        }
#else
        std::error_code replaceError;
        std::filesystem::rename(temporaryPath, filePath, replaceError);
        if(replaceError) {
            std::error_code ignored;
            std::filesystem::remove(temporaryPath, ignored);
            return std::unexpected(std::ios_base::failure(
                std::format("failed to replace '{}': {}", filePath.string(), replaceError.message())));
        }
#endif

        return {};
    }
}
