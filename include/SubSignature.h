#ifndef SUBSIGNATURE_H
#define SUBSIGNATURE_H

#include <string_view>

#include <Statement.h>
#include <Expression.h>

namespace ngc {
    class SubSignature {
        std::string_view m_name;
        size_t m_numParams;

    public:
        SubSignature(const std::string_view name, const size_t numParams) : m_name(name), m_numParams(numParams) { }
        explicit SubSignature(const SubStatement *stmt) : m_name(stmt->name()), m_numParams(stmt->params().size()) { }
        explicit SubSignature(const CallExpression *expr) : m_name(expr->name()), m_numParams(expr->args().size()) { }
        bool operator==(const SubSignature& s) const { return s.m_name == m_name && s.m_numParams == m_numParams; }
        [[nodiscard]] std::string_view name() const { return m_name; }
        [[nodiscard]] size_t numParams() const { return m_numParams; }
        [[nodiscard]] std::string toString() const { return std::format("{}[{}]", name(), numParams()); }
    };
}

template<>
struct std::hash<ngc::SubSignature> {
    size_t operator()(const ngc::SubSignature &s) const noexcept {
        const auto h1 = std::hash<std::string_view>{}(s.name());
        const auto h2 = std::hash<size_t>{}(s.numParams());
        return h1 ^ (h2 << 1);
    }
};

#endif //SUBSIGNATURE_H
