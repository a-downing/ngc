#ifndef TOKENSOURCE_H
#define TOKENSOURCE_H

#include <memory>
#include <string_view>

namespace ngc {
    class TokenSource {
    public:
        virtual ~TokenSource() = default;
        [[nodiscard]] virtual std::string_view text() const = 0;
        [[nodiscard]] virtual std::string_view name() const = 0;
        [[nodiscard]] virtual int line() const = 0;
        [[nodiscard]] virtual int col() const = 0;
        [[nodiscard]] virtual std::unique_ptr<TokenSource> clone() const = 0;
    };
}

#endif //TOKENSOURCE_H
