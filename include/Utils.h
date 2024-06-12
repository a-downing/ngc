#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <iterator>
#include <numeric>
#include <filesystem>
#include <fstream>

namespace ngc
{
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

    template <typename T>
    std::string join(const T &c, const std::string &sep) {
        if(c.size() == 0) {
            return "";
        }

        return std::accumulate(std::next(c.cbegin()), c.cend(), c.front(), [&sep](const auto &_a, const auto &_b) {
            return _a + sep + _b;
        });
    }
}

#endif //UTILS_H
