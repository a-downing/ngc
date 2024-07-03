#include <cstddef>
#include <print>
#include <string>
#include <numbers>

#include "utils.h"

struct vec3 {
    double x, y, z;
};

template <>
struct std::formatter<vec3> {
    constexpr auto parse(std::format_parse_context& ctx) {
        return ctx.begin();
    }

    auto format(const vec3& v, std::format_context& ctx) const {
        return std::format_to(ctx.out(), "vec3({:.3f}, {}, {})", v.x, v.y, v.z);
    }
};

int consteval test() {
    std::vector<int> v;
    v.push_back(42);
    return v.back();
}

size_t consteval test2() {
    std::string s;
    s += "Hello World!";
    return s.size();
}

int main() {
    auto v = vec3{1, 2, 3};

    std::println("{}", v);
    std::println("pi: {}", std::numbers::pi_v<float>);
    std::println("pi: {}", std::numbers::pi_v<double>);
    std::println("test(): {}", test());
    std::println("test2(): {}", test2());
}