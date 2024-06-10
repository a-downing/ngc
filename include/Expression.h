
#ifndef EXPRESSION_H
#define EXPRESSION_H

#include <memory>
#include <utility>

#include <Utils.h>
#include <Token.h>
#include <Visitor.h>

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

        virtual bool is(const class CommentExpression *expression) const { return false; }
        virtual bool is(const class WordExpression *expression) const { return false; }
        virtual bool is(const class RealExpression *expression) const { return false; }
        virtual bool is(const class LiteralExpression *expression) const { return false; }
        virtual bool is(const class VariableExpression *expression) const { return false; }
        virtual bool is(const class NumericVariableExpression *expression) const { return false; }
        virtual bool is(const class NamedVariableExpression *expression) const { return false; }
        virtual bool is(const class UnaryExpression *expression) const { return false; }
        virtual bool is(const class BinaryExpression *expression) const { return false; }
        virtual bool is(const class CallExpression *expression) const { return false; }
        virtual bool is(const class GroupingExpression *expression) const { return false; }

        template<typename T>
        [[nodiscard]] bool is() const {
            return is(static_cast<const T *>(this));
        }

        template<typename T>
        const T *as() const {
            return is(static_cast<const T *>(this)) ? static_cast<const T *>(this) : nullptr;
        }

        [[nodiscard]] virtual constexpr const char *className() const = 0;
        [[nodiscard]] virtual std::string toString() const = 0;
        virtual void accept(Visitor &v, VisitorContext *ctx) const = 0;
    };

    class CommentExpression final : public Expression {
    public:
        explicit CommentExpression(const Token &token) : Expression(token) { }
        ~CommentExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        bool is(const CommentExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "CommentExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::string(token().text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class RealExpression : public Expression {
    public:
        explicit RealExpression(const Token &token) : Expression(token) { }
        [[nodiscard]] static constexpr const char *staticClassName() { return "RealExpression"; }
    };

    class WordExpression final : public Expression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        WordExpression(const Token &token, std::unique_ptr<RealExpression> real): Expression(token), m_realExpression(std::move(real)) { }
        ~WordExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        bool is(const WordExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "WordExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class LiteralExpression final : public RealExpression {
    public:
        explicit LiteralExpression(const Token &token) : RealExpression(token) { }
        ~LiteralExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        bool is(const RealExpression *expression) const override { return true; }
        bool is(const LiteralExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "LiteralExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::string(token().text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class VariableExpression : public RealExpression {
    public:
        explicit VariableExpression(const Token &token) : RealExpression(token) { }
        [[nodiscard]] static constexpr const char *staticClassName() { return "VariableExpression"; }
    };

    class NumericVariableExpression final : public VariableExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        explicit NumericVariableExpression(const Token &token, std::unique_ptr<RealExpression> realExpression) : VariableExpression(token), m_realExpression(std::move(realExpression)) { }
        ~NumericVariableExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        bool is(const RealExpression *expression) const override { return true; }
        bool is(const VariableExpression *expression) const override { return true; }
        bool is(const NumericVariableExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "NumericVariableExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class NamedVariableExpression final : public VariableExpression {
    public:
        explicit NamedVariableExpression(const Token &token) : VariableExpression(token) { }
        ~NamedVariableExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        bool is(const RealExpression *expression) const override { return true; }
        bool is(const VariableExpression *expression) const override { return true; }
        bool is(const NamedVariableExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "NamedVariableExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::string(token().text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        [[nodiscard]] std::string_view name() const { return token().value(); }
    };

    class UnaryExpression final : public RealExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        explicit UnaryExpression(const Token &token, std::unique_ptr<RealExpression> realExpression) : RealExpression(token), m_realExpression(std::move(realExpression)) { }
        ~UnaryExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        bool is(const RealExpression *expression) const override { return true; }
        bool is(const UnaryExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "UnaryExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
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

        explicit BinaryExpression(const Token &token, std::unique_ptr<RealExpression> left, std::unique_ptr<RealExpression> right) : RealExpression(token), m_left(std::move(left)), m_right(std::move(right)) { }
        ~BinaryExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return m_left->startToken(); }
        [[nodiscard]] const Token &endToken() const override { return m_right->endToken(); }
        bool is(const RealExpression *expression) const override { return true; }
        bool is(const BinaryExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "BinaryExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::format("[{} {} {}]", m_left->text(), token().text(), m_right->text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        [[nodiscard]] const std::unique_ptr<RealExpression> &left() const { return m_left; }
        [[nodiscard]] const std::unique_ptr<RealExpression> &right() const { return m_right; }

        [[nodiscard]] Op op() const {
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
                default: throw LogicError(std::format("invalid token {} '{}' for BinaryExpression", token().name(), token().text()));
            }
        }

        [[nodiscard]] std::string_view opName() const {
            switch (token().kind()) {
                case Token::Kind::ASSIGN: return "ASSIGN";
                case Token::Kind::EQ: return "EQ";
                case Token::Kind::NE: return "NE";
                case Token::Kind::LT: return "LT";
                case Token::Kind::LE: return "LE";
                case Token::Kind::GT: return "GT";
                case Token::Kind::GE: return "GE";
                case Token::Kind::PLUS: return "ADD";
                case Token::Kind::MINUS: return "SUB";
                case Token::Kind::MUL: return "MUL";
                case Token::Kind::SLASH: return "DIV";
                case Token::Kind::MOD: return "MOD";
                case Token::Kind::AND: return "AND";
                case Token::Kind::OR: return "OR";
                case Token::Kind::XOR: return "XOR";
                default: throw LogicError(std::format("invalid token {} '{}' for BinaryExpression", token().name(), token().text()));
            }
        }
    };

    class CallExpression final : public RealExpression {
        Token m_endToken;
        std::vector<std::unique_ptr<RealExpression>> m_args;

    public:
        explicit CallExpression(const Token &token, const Token &endToken, std::vector<std::unique_ptr<RealExpression>> args) : RealExpression(token), m_endToken(endToken), m_args(std::move(args)) { }
        ~CallExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        bool is(const RealExpression *expression) const override { return true; }
        bool is(const CallExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "CallExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }

        [[nodiscard]] std::string_view name() const { return token().value(); }

        [[nodiscard]] std::string toString() const override {
            std::vector<std::string> args;

            for(const auto &arg : m_args) {
                args.emplace_back(arg->text());
            }

            return std::format("{}[{}]", token().text(), join(args, ", "));
        }

        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class GroupingExpression final : public RealExpression {
        Token m_endToken;
        std::unique_ptr<RealExpression> m_expression;

    public:
        explicit GroupingExpression(const Token &token, const Token &endToken, std::unique_ptr<RealExpression> expression) : RealExpression(token), m_endToken(endToken), m_expression(std::move(expression)) { }
        ~GroupingExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        bool is(const RealExpression *expression) const override { return true; }
        bool is(const GroupingExpression *expression) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "GroupingExpression"; }

        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        [[nodiscard]] std::string toString() const override { return std::string(m_expression->text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };
}

#endif //EXPRESSION_H
