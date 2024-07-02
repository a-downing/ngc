#include <cstddef>
#include <print>
#include <string>


#include "utils.h"

int main() {
    auto result = ngc::readFile("autoload/tool_change.ngc");

    if(!result) {
        throw result.error();
    }

    auto text = *result;

    std::println("text.size(): {}", text.size());

    for(size_t i = 0; i < text.size(); i++) {
        std::println("{}: 0x{:02X} '{}'", i, text[i], text[i]);
    }
}