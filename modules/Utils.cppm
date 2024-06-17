module;

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
    inline std::string toChars(const double d) {
        std::array<char, 32> chars;
        auto [ptr, ec] = std::to_chars(chars.data(), chars.data() + chars.size(), d);

        if(ec != std::errc()) {
            throw std::runtime_error(std::format("{} failed to convert {}", __func__, d));
        }

        return std::string(chars.data(), ptr);
    }

    inline std::string readFile(const std::filesystem::path& filePath) {
        std::ifstream file(filePath);

        if (!file) {
            throw std::ios_base::failure("Failed to open file");
        }

        file.seekg(0, std::ios::end);
        const auto fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        std::string fileContent(fileSize, '\0');
        file.read(&fileContent[0], fileSize);

        return fileContent;
    }
}
