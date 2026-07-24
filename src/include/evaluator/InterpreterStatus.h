#pragma once

#include <string>

namespace ngc {
    enum class InterpreterStatusKind {
        Alert,
        Print,
        Error,
    };

    struct InterpreterStatusMessage {
        InterpreterStatusKind kind;
        std::string text;
    };
}
