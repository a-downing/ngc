# NGC C++ Style Guide

Apply these conventions to new and modified C++ code. Keep unrelated surrounding code unchanged unless a broader formatting pass is explicitly requested.

## Control flow

Put one space between a control-flow keyword and its opening parenthesis:

```cpp
if (ready) {
    run();
}

for (std::size_t index = 0; index < count; ++index) {
    consume(index);
}

switch (axis) {
    case Axis::X: return 'X';
    case Axis::Y: return 'Y';
}
```

Always use braces for `if`, `else`, `for`, and `while` bodies, including single-statement bodies. Keep `else` on the same line as the preceding closing brace:

```cpp
if (result.size() > width) {
    result.resize(width);
} else {
    result.append(width - result.size(), ' ');
}
```

Ordinary function calls do not have a space before their opening parenthesis.

## Layout

Use four spaces for indentation and do not use tabs. Use blank lines to separate logical steps within a function, including setup, mutation, fallback handling, and the final result. In particular, separate initial setup from the operation that follows, and separate a final return value from preceding side effects. A function containing only setup followed by a return does not require a blank line between them.

Prefer keeping function declarations, function definitions, and conditions on one line, even when they are relatively long. Split a line only when keeping it whole would materially hurt readability.

Keep a constructor's initializer list on the same line as its signature when practical:

```cpp
Manager::Manager(std::unique_ptr<Driver> driver, const std::chrono::milliseconds reportTimeout) : m_driver(std::move(driver)), m_reportTimeout(reportTimeout) {
    start();
}
```

Compact switch cases may keep a short action on the same line as the label, as in `case Axis::X: return 'X';`.

Do not leave trailing whitespace on any line. Preserve the file's existing newline convention and avoid formatting unrelated lines.
