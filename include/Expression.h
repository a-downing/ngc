
#ifndef EXPRESSION_H
#define EXPRESSION_H

#include <memory>
#include <utility>

#include <Utils.h>
#include <Token.h>

namespace ngc
{
    class Expression {
        Token m_token;

    public:
        explicit Expression(const Token &token) : m_token(token) { }
        virtual ~Expression() = default;

        [[nodiscard]] const Token &token() const { return m_token; }

        [[nodiscard]] virtual const Token &startToken() const = 0;
        [[nodiscard]] virtual const Token &endToken() const = 0;

        [[nodiscard]] std::string_view text() const { return m_token.source()->text(startToken().start(), endToken().end()); }

        virtual bool is(class CommentExpression *expression) const { return false; }
        virtual bool is(class WordExpression *expression) const { return false; }
        virtual bool is(class RealExpression *expression) const { return false; }
        virtual bool is(class LiteralExpression *expression) const { return false; }
        virtual bool is(class VariableExpression *expression) const { return false; }
        virtual bool is(class NumericVariableExpression *expression) const { return false; }
        virtual bool is(class NamedVariableExpression *expression) const { return false; }
        virtual bool is(class UnaryExpression *expression) const { return false; }
        virtual bool is(class BinaryExpression *expression) const { return false; }
        virtual bool is(class CallExpression *expression) const { return false; }
        virtual bool is(class GroupingExpression *expression) const { return false; }

        template<typename T>
        bool is() {
            return is(static_cast<T *>(this));
        }

        [[nodiscard]] virtual constexpr const char *name() const = 0;
        [[nodiscard]] virtual std::string toString() const = 0;
    };

    class CommentExpression final : public Expression {
    public:
        explicit CommentExpression(const Token &token) : Expression(token) { }
        ~CommentExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        bool is(CommentExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "CommentExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::string(token().text()); }
    };

    class RealExpression : public Expression {
    public:
        explicit RealExpression(const Token &token) : Expression(token) { }
        [[nodiscard]] static constexpr const char *staticName() { return "RealExpression"; }
    };

    class WordExpression final : public Expression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        WordExpression(const Token &token, std::unique_ptr<RealExpression> real): Expression(token), m_realExpression(std::move(real)) { }
        ~WordExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        bool is(WordExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "WordExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
    };

    class LiteralExpression final : public RealExpression {
    public:
        explicit LiteralExpression(const Token &token) : RealExpression(token) { }
        ~LiteralExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        bool is(RealExpression *expression) const override { return true; }
        bool is(LiteralExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "LiteralExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::string(token().text()); }
    };

    class VariableExpression : public RealExpression {
    public:
        explicit VariableExpression(const Token &token) : RealExpression(token) { }
        [[nodiscard]] static constexpr const char *staticName() { return "VariableExpression"; }
    };

    class NumericVariableExpression final : public VariableExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        explicit NumericVariableExpression(const Token &token, std::unique_ptr<RealExpression> realExpression) : VariableExpression(token), m_realExpression(std::move(realExpression)) { }
        ~NumericVariableExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        bool is(RealExpression *expression) const override { return true; }
        bool is(VariableExpression *expression) const override { return true; }
        bool is(NumericVariableExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "NumericVariableExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
    };

    class NamedVariableExpression final : public VariableExpression {
    public:
        explicit NamedVariableExpression(const Token &token) : VariableExpression(token) { }
        ~NamedVariableExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        bool is(RealExpression *expression) const override { return true; }
        bool is(VariableExpression *expression) const override { return true; }
        bool is(NamedVariableExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "NamedVariableExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::string(token().text()); }
    };

    class UnaryExpression final : public RealExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        explicit UnaryExpression(const Token &token, std::unique_ptr<RealExpression> realExpression) : RealExpression(token), m_realExpression(std::move(realExpression)) { }
        ~UnaryExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        bool is(RealExpression *expression) const override { return true; }
        bool is(UnaryExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "UnaryExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
    };

    class BinaryExpression final : public RealExpression {
        std::unique_ptr<RealExpression> m_left;
        std::unique_ptr<RealExpression> m_right;

    public:
        explicit BinaryExpression(const Token &token, std::unique_ptr<RealExpression> left, std::unique_ptr<RealExpression> right) : RealExpression(token), m_left(std::move(left)), m_right(std::move(right)) { }
        ~BinaryExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return m_left->startToken(); }
        [[nodiscard]] const Token &endToken() const override { return m_right->endToken(); }
        bool is(RealExpression *expression) const override { return true; }
        bool is(BinaryExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "BinaryExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::format("[{} {} {}]", m_left->text(), token().text(), m_right->text()); }
    };

    class CallExpression final : public RealExpression {
        Token m_endToken;
        std::vector<std::unique_ptr<RealExpression>> m_args;

    public:
        explicit CallExpression(const Token &token, const Token &endToken, std::vector<std::unique_ptr<RealExpression>> args) : RealExpression(token), m_endToken(endToken), m_args(std::move(args)) { }
        ~CallExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        bool is(RealExpression *expression) const override { return true; }
        bool is(CallExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "CallExpression"; }
        [[nodiscard]] constexpr const char *name() const override { return staticName(); }

        [[nodiscard]] std::string toString() const override {
            std::vector<std::string> args;

            for(const auto &arg : m_args) {
                args.emplace_back(arg->text());
            }

            return std::format("{}[{}]", token().text(), join(args, ", "));
        }
    };

    class GroupingExpression final : public RealExpression {
        Token m_endToken;
        std::unique_ptr<RealExpression> m_expression;

    public:
        explicit GroupingExpression(const Token &token, const Token &endToken, std::unique_ptr<RealExpression> expression) : RealExpression(token), m_endToken(endToken), m_expression(std::move(expression)) { }
        ~GroupingExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        bool is(RealExpression *expression) const override { return true; }
        bool is(GroupingExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticName() { return "GroupingExpression"; }

        [[nodiscard]] constexpr const char *name() const override { return staticName(); }
        [[nodiscard]] std::string toString() const override { return std::string(m_expression->text()); }
    };
}

#endif //EXPRESSION_H
