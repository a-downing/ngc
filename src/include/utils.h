#pragma once

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <format>
#include <print>
#include <string>
#include <filesystem>
#include <fstream>
#include <charconv>
#include <array>
#include <source_location>

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

        return d;
    }

    inline std::expected<std::string, std::ios_base::failure> readFile(const std::filesystem::path& filePath) {
        std::ifstream file(filePath, std::ios::binary);

        if (!file) {
            return std::unexpected(std::ios_base::failure("Failed to open file"));
        }

        file.seekg(0, std::ios::end);
        const auto fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        std::println("readFile({}) fileSize: {}", filePath.string(), (size_t)fileSize);

        std::string fileContent(fileSize, '\0');
        file.read(&fileContent[0], fileSize);

        return fileContent;
    }

    inline std::expected<void, std::ios_base::failure> writeFile(const std::filesystem::path& filePath, const std::string_view text) {
        std::ofstream file(filePath);

        if (!file) {
            return std::unexpected(std::ios_base::failure("Failed to open file"));
        }

        file.write(text.data(), text.size());

        return {};
    }
}
