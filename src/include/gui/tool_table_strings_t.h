#pragma once

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <string>

#include "utils.h"
#include "machine/ToolTable.h"

struct tool_table_strings_t {
    std::string number;
    std::string x, y, z, a, b, c;
    std::string diameter;
    std::string comment;

    static tool_table_strings_t from(const ngc::ToolTable::tool_entry_t &tool) {
        return {
            ngc::toChars(tool.number),
            ngc::toChars(tool.x),
            ngc::toChars(tool.y),
            ngc::toChars(tool.z),
            ngc::toChars(tool.a),
            ngc::toChars(tool.b),
            ngc::toChars(tool.c),
            ngc::toChars(tool.diameter),
            tool.comment
        };
    }

    std::expected<ngc::ToolTable::tool_entry_t, std::string> to() const {
        auto _number = ngc::fromChars(number);
        if(!_number) {  return std::unexpected(std::format("failed to convert '{}' to number", number)); }

        auto _x = ngc::fromChars(x);
        if(!_x) {  return std::unexpected(std::format("failed to convert '{}' to number", x)); }

        auto _y = ngc::fromChars(y);
        if(!_y) {  return std::unexpected(std::format("failed to convert '{}' to number", y)); }

        auto _z = ngc::fromChars(z);
        if(!_z) {  return std::unexpected(std::format("failed to convert '{}' to number", z)); }

        auto _a = ngc::fromChars(a);
        if(!_a) {  return std::unexpected(std::format("failed to convert '{}' to number", a)); }

        auto _b = ngc::fromChars(b);
        if(!_b) {  return std::unexpected(std::format("failed to convert '{}' to number", b)); }

        auto _c = ngc::fromChars(c);
        if(!_c) {  return std::unexpected(std::format("failed to convert '{}' to number", c)); }

        auto _diameter = ngc::fromChars(diameter);
        if(!_diameter) {  return std::unexpected(std::format("failed to convert '{}' to number", diameter)); }
        
        return ngc::ToolTable::tool_entry_t { static_cast<int>(*_number), *_x, *_y, *_z, *_a, *_b, *_c, *_diameter, comment };
    }
};