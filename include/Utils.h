#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <iterator>
#include <numeric>

namespace ngc
{
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
