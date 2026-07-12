#pragma once

#include <string>

namespace ngc {
    enum class InterpreterStatusKind {
        Print,
        Error,
    };

    struct InterpreterStatusMessage {
        InterpreterStatusKind kind;
        std::string text;
    };
}
