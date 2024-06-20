module;

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <format>
#include <string>
#include <iterator>
#include <filesystem>
#include <fstream>
#include <charconv>
#include <array>

export module utils;

export namespace ngc
{
    std::string toChars(const double d) {
        std::array<char, 32> chars;
        auto [ptr, ec] = std::to_chars(chars.data(), chars.data() + chars.size(), d);

        if(ec != std::errc()) {
            throw std::runtime_error(std::format("{}() failed to convert {}", __func__, d));
        }

        return std::string(chars.data(), ptr);
    }

    std::expected<double, std::errc> fromChars(const std::string_view text) {
        double d;
        auto [ptr, ec] = std::from_chars(text.begin(), text.end(), d);

        if(ec != std::errc()) {
            return std::unexpected(ec);
        }

        return d;
    }

    std::expected<std::string, std::ios_base::failure> readFile(const std::filesystem::path& filePath) {
        std::ifstream file(filePath);

        if (!file) {
            return std::unexpected(std::ios_base::failure("Failed to open file"));
        }

        file.seekg(0, std::ios::end);
        const auto fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        std::string fileContent(fileSize, '\0');
        file.read(&fileContent[0], fileSize);

        return fileContent;
    }

    std::expected<void, std::ios_base::failure> writeFile(const std::filesystem::path& filePath, const std::string_view text) {
        std::ofstream file(filePath);

        if (!file) {
            return std::unexpected(std::ios_base::failure("Failed to open file"));
        }

        file.write(text.data(), text.size());

        return {};
    }
}
