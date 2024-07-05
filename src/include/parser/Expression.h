#pragma once

#include <utility>
#include <vector>
#include <memory>
#include <utility>

#include "parser/Token.h"
#include "parser/Visitor.h"

namespace ngc
{
    class Expression { 
        Token m_token;

    public:
        explicit Expression(Token  token) : m_token(std::move(token)) { }
        virtual ~Expression() = default;

        const Token &token() const { return m_token; }

        virtual const Token &startToken() const = 0;
        virtual const Token &endToken() const = 0;
        virtual constexpr const char *className() const = 0;
        virtual std::string text() const = 0;

        virtual bool isImpl(const class CommentExpression *) const { return false; }
        virtual bool isImpl(const class WordExpression *) const { return false; }
        virtual bool isImpl(const class ScalarExpression *) const { return false; }
        virtual bool isImpl(const class RealExpression *) const { return false; }
        virtual bool isImpl(const class LiteralExpression *) const { return false; }
        virtual bool isImpl(const class StringExpression *) const { return false; }
        virtual bool isImpl(const class VariableExpression *) const { return false; }
        virtual bool isImpl(const class NumericVariableExpression *) const { return false; }
        virtual bool isImpl(const class NamedVariableExpression *) const { return false; }
        virtual bool isImpl(const class UnaryExpression *) const { return false; }
        virtual bool isImpl(const class BinaryExpression *) const { return false; }
        virtual bool isImpl(const class CallExpression *) const { return false; }
        virtual bool isImpl(const class GroupingExpression *) const { return false; }

        template<typename T>
        bool is() const requires std::derived_from<T, Expression> {
            return this->isImpl(static_cast<const T *>(nullptr));
        }

        template<typename T>
        const T *as() const requires std::derived_from<T, Expression> {
            return this->isImpl(static_cast<const T *>(nullptr)) ? static_cast<const T *>(this) : nullptr;
        }

        virtual void accept(Visitor &v, VisitorContext *ctx) const = 0;
    };

    template<typename Expr>
    inline std::string join(const std::vector<std::unique_ptr<Expr>> &expressions, const std::string_view sep) {
        std::string result;

        for(const auto &expr : expressions) {
            result += expr->text();

            if(expr != expressions.back()) {
                result += sep;
            }
        }

        return result;
    }

    class CommentExpression final : public Expression {
    public:
        using Expression::is;
        explicit CommentExpression(Token token) : Expression(std::move(token)) { }
        ~CommentExpression() override = default;

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return token(); }
        std::string text() const override { return std::string(token().text()); }

        bool isImpl(const CommentExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "CommentExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class ScalarExpression : public Expression {
    public:
        explicit ScalarExpression(Token token) : Expression(std::move(token)) { }
        bool isImpl(const ScalarExpression *) const final { return true; }
        static constexpr const char *staticClassName() { return "ScalarExpression"; }
    };

    class RealExpression : public ScalarExpression {
    public:
    using ScalarExpression::isImpl;
        explicit RealExpression(Token token) : ScalarExpression(std::move(token)) { }
        bool isImpl(const RealExpression *) const final { return true; }
        static constexpr const char *staticClassName() { return "RealExpression"; }
    };

    class WordExpression final : public Expression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        WordExpression(Token token, std::unique_ptr<RealExpression> real): Expression(std::move(token)), m_realExpression(std::move(real)) { }
        ~WordExpression() override = default;

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return m_realExpression->endToken(); }
        std::string text() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        const RealExpression *real() const { return m_realExpression.get(); }

        bool isImpl(const WordExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "WordExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class LiteralExpression final : public RealExpression {
    public:
        explicit LiteralExpression(Token token) : RealExpression(std::move(token)) { }
        ~LiteralExpression() override = default;

        static std::unique_ptr<LiteralExpression> fromDouble(const double d) { return std::make_unique<LiteralExpression>(Token::fromDouble(d)); }

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return token(); }
        std::string text() const override { return std::string(token().text()); }
        double value() const { return token().as_double(); }

        using RealExpression::isImpl;
        bool isImpl(const LiteralExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "LiteralExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class StringExpression final : public ScalarExpression {
    public:
        explicit StringExpression(Token token) : ScalarExpression(std::move(token)) { }
        ~StringExpression() override = default;

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return token(); }
        std::string text() const override { return std::string(token().text()); }
        std::string_view value() const { return token().value(); }

        using ScalarExpression::isImpl;
        bool isImpl(const StringExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "StringExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class VariableExpression : public RealExpression {
    public:
        explicit VariableExpression(Token token) : RealExpression(std::move(token)) { }
        static constexpr const char *staticClassName() { return "VariableExpression"; }
        using RealExpression::isImpl;
        bool isImpl(const VariableExpression *) const final { return true; }
    };

    class NumericVariableExpression final : public VariableExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        explicit NumericVariableExpression(Token token, std::unique_ptr<RealExpression> realExpression) : VariableExpression(std::move(token)), m_realExpression(std::move(realExpression)) { }
        ~NumericVariableExpression() override = default;

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return m_realExpression->endToken(); }
        std::string text() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        const RealExpression *real() const { return m_realExpression.get(); }

        using VariableExpression::isImpl;
        bool isImpl(const NumericVariableExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "NumericVariableExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class NamedVariableExpression final : public VariableExpression {
    public:
        explicit NamedVariableExpression(Token token) : VariableExpression(std::move(token)) { }
        ~NamedVariableExpression() override = default;

        static std::unique_ptr<NamedVariableExpression> fromName(std::string name) {
            return std::make_unique<NamedVariableExpression>(Token::fromString(Token::Kind::NAMED_VARIABLE, std::format("#{}", name)));
        }

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return token(); }
        std::string text() const override { return std::string(token().text()); }

        using VariableExpression::isImpl;
        bool isImpl(const NamedVariableExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "NamedVariableExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        std::string_view name() const { return token().value(); }
    };

    class UnaryExpression final : public RealExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        enum class Op {
            ADDRESS_OF,
            NEGATIVE,
            POSITIVE
        };

        explicit UnaryExpression(Token token, std::unique_ptr<RealExpression> realExpression) : RealExpression(std::move(token)), m_realExpression(std::move(realExpression)) { }
        ~UnaryExpression() override = default;

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return m_realExpression->endToken(); }
        std::string text() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        const RealExpression *real() const { return m_realExpression.get(); }

        using RealExpression::isImpl;
        bool isImpl(const UnaryExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "UnaryExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        Op op() const {
            switch (token().kind()) {
            case Token::Kind::AMPERSAND: return Op::ADDRESS_OF;
            case Token::Kind::MINUS: return Op::NEGATIVE;
            case Token::Kind::PLUS: return Op::POSITIVE;
            default: throw std::logic_error(std::format("invalid token {} '{}' for UnaryExpression", token().name(), token().text()));
            }
        }
    };

    class BinaryExpression final : public RealExpression {
        std::unique_ptr<RealExpression> m_left;
        std::unique_ptr<RealExpression> m_right;

    public:
        enum class Op {
            ASSIGN,
            AND, OR, XOR,
            EQ, NE, LT, LE, GT, GE,
            ADD, SUB,
            MUL, DIV, MOD,
        };

        explicit BinaryExpression(Token token, std::unique_ptr<RealExpression> left, std::unique_ptr<RealExpression> right) : RealExpression(std::move(token)), m_left(std::move(left)), m_right(std::move(right)) { }
        ~BinaryExpression() override = default;

        const Token &startToken() const override { return m_left->startToken(); }
        const Token &endToken() const override { return m_right->endToken(); }
        std::string text() const override { return std::format("[{} {} {}]", m_left->text(), token().text(), m_right->text()); }

        using RealExpression::isImpl;
        bool isImpl(const BinaryExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "BinaryExpression"; }
        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        const RealExpression *left() const { return m_left.get(); }
        const RealExpression *right() const { return m_right.get(); }

        Op op() const {
            switch (token().kind()) {
                case Token::Kind::ASSIGN: return Op::ASSIGN;
                case Token::Kind::EQ: return Op::EQ;
                case Token::Kind::NE: return Op::NE;
                case Token::Kind::LT: return Op::LT;
                case Token::Kind::LE: return Op::LE;
                case Token::Kind::GT: return Op::GT;
                case Token::Kind::GE: return Op::GE;
                case Token::Kind::PLUS: return Op::ADD;
                case Token::Kind::MINUS: return Op::SUB;
                case Token::Kind::MUL: return Op::MUL;
                case Token::Kind::SLASH: return Op::DIV;
                case Token::Kind::MOD: return Op::MOD;
                case Token::Kind::AND: return Op::AND;
                case Token::Kind::OR: return Op::OR;
                case Token::Kind::XOR: return Op::XOR;
                default: throw std::logic_error(std::format("invalid token {} '{}' for BinaryExpression", token().name(), token().text()));
            }
        }
    };

    class CallExpression final : public RealExpression {
        Token m_endToken;
        std::vector<std::unique_ptr<ScalarExpression>> m_args;

    public:
        explicit CallExpression(Token token, Token endToken, std::vector<std::unique_ptr<ScalarExpression>> args) : RealExpression(std::move(token)), m_endToken(std::move(endToken)), m_args(std::move(args)) { }
        ~CallExpression() override = default;

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return m_endToken; }
        std::string text() const override { return std::format("{}[{}]", token().text(), join(m_args, ", ")); }

        using RealExpression::isImpl;
        bool isImpl(const CallExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "CallExpression"; }
        constexpr const char *className() const override { return staticClassName(); }

        std::string_view name() const { return token().value(); }
        const std::vector<std::unique_ptr<ScalarExpression>> &args() const { return m_args; }

        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class GroupingExpression final : public RealExpression {
        Token m_endToken;
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        explicit GroupingExpression(Token token, Token endToken, std::unique_ptr<RealExpression> expression) : RealExpression(std::move(token)), m_endToken(std::move(endToken)), m_realExpression(std::move(expression)) { }
        ~GroupingExpression() override = default;

        const Token &startToken() const override { return token(); }
        const Token &endToken() const override { return m_endToken; }
        std::string text() const override { return std::format("[{}]", m_realExpression->text()); }
        const RealExpression *real() const { return m_realExpression.get(); }

        using RealExpression::isImpl;
        bool isImpl(const GroupingExpression *) const override { return true; }

        static constexpr const char *staticClassName() { return "GroupingExpression"; }

        constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };
}
