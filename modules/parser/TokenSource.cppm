module;

#include <memory>
#include <string_view>

export module parser:TokenSource;

export namespace ngc {
    class TokenSource {
    public:
        [[nodiscard]] virtual std::unique_ptr<TokenSource> clone() const = 0;
        virtual ~TokenSource() = default;
        [[nodiscard]] virtual std::string_view text() const = 0;
        [[nodiscard]] virtual std::string_view name() const = 0;
        [[nodiscard]] virtual int line() const = 0;
        [[nodiscard]] virtual int col() const = 0;
        [[nodiscard]] virtual std::string location() const = 0;
    };
}
