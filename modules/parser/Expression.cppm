module;

#include <utility>
#include <vector>
#include <memory>
#include <utility>
#include <stdexcept>

export module parser:Expression;
import :Token;
import :Visitor;

export namespace ngc
{
    class Expression {
        Token m_token;

    public:
        explicit Expression(Token  token) : m_token(std::move(token)) { }
        virtual ~Expression() = default;

        [[nodiscard]] const Token &token() const { return m_token; }

        [[nodiscard]] virtual const Token &startToken() const = 0;
        [[nodiscard]] virtual const Token &endToken() const = 0;
        [[nodiscard]] virtual constexpr const char *className() const = 0;
        [[nodiscard]] virtual std::string text() const = 0;

        virtual bool is(const class CommentExpression *) const { return false; }
        virtual bool is(const class WordExpression *) const { return false; }
        virtual bool is(const class ScalarExpression *) const { return false; }
        virtual bool is(const class RealExpression *) const { return false; }
        virtual bool is(const class LiteralExpression *) const { return false; }
        virtual bool is(const class StringExpression *) const { return false; }
        virtual bool is(const class VariableExpression *) const { return false; }
        virtual bool is(const class NumericVariableExpression *) const { return false; }
        virtual bool is(const class NamedVariableExpression *) const { return false; }
        virtual bool is(const class UnaryExpression *) const { return false; }
        virtual bool is(const class BinaryExpression *) const { return false; }
        virtual bool is(const class CallExpression *) const { return false; }
        virtual bool is(const class GroupingExpression *) const { return false; }

        template<typename T>
        [[nodiscard]] bool is() const {
            return this->is(static_cast<const T *>(this));
        }

        template<typename T>
        const T *as() const {
            return this->is(static_cast<const T *>(this)) ? static_cast<const T *>(this) : nullptr;
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

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        [[nodiscard]] std::string text() const override { return std::string(token().text()); }

        bool is(const CommentExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "CommentExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class ScalarExpression : public Expression {
    public:
        using Expression::is;
        explicit ScalarExpression(Token token) : Expression(std::move(token)) { }
        bool is(const ScalarExpression *) const final { return true; }
        [[nodiscard]] static constexpr const char *staticClassName() { return "ScalarExpression"; }
    };

    class RealExpression : public ScalarExpression {
    public:
        using Expression::is;
        explicit RealExpression(Token token) : ScalarExpression(std::move(token)) { }
        bool is(const RealExpression *) const final { return true; }
        [[nodiscard]] static constexpr const char *staticClassName() { return "RealExpression"; }
    };

    class WordExpression final : public Expression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        using Expression::is;
        WordExpression(Token token, std::unique_ptr<RealExpression> real): Expression(std::move(token)), m_realExpression(std::move(real)) { }
        ~WordExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        [[nodiscard]] const RealExpression *real() const { return m_realExpression.get(); }

        bool is(const WordExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "WordExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class LiteralExpression final : public RealExpression {
    public:
        using Expression::is;
        explicit LiteralExpression(Token token) : RealExpression(std::move(token)) { }
        ~LiteralExpression() override = default;

        static std::unique_ptr<LiteralExpression> fromDouble(const double d) { return std::make_unique<LiteralExpression>(Token::fromDouble(d)); }

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        [[nodiscard]] std::string text() const override { return std::string(token().text()); }
        [[nodiscard]] double value() const { return token().as_double(); }

        bool is(const LiteralExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "LiteralExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class StringExpression final : public ScalarExpression {
    public:
        using Expression::is;
        explicit StringExpression(Token token) : ScalarExpression(std::move(token)) { }
        ~StringExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        [[nodiscard]] std::string text() const override { return std::string(token().text()); }
        [[nodiscard]] std::string_view value() const { return token().value(); }

        bool is(const StringExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "StringExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class VariableExpression : public RealExpression {
    public:
        using Expression::is;
        explicit VariableExpression(Token token) : RealExpression(std::move(token)) { }
        [[nodiscard]] static constexpr const char *staticClassName() { return "VariableExpression"; }

        bool is(const VariableExpression *) const final { return true; }
    };

    class NumericVariableExpression final : public VariableExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        using Expression::is;
        explicit NumericVariableExpression(Token token, std::unique_ptr<RealExpression> realExpression) : VariableExpression(std::move(token)), m_realExpression(std::move(realExpression)) { }
        ~NumericVariableExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        [[nodiscard]] const RealExpression *real() const { return m_realExpression.get(); }

        bool is(const NumericVariableExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "NumericVariableExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class NamedVariableExpression final : public VariableExpression {
    public:
        using Expression::is;
        explicit NamedVariableExpression(Token token) : VariableExpression(std::move(token)) { }
        ~NamedVariableExpression() override = default;

        static std::unique_ptr<NamedVariableExpression> fromName(std::string name) {
            return std::make_unique<NamedVariableExpression>(Token::fromString(Token::Kind::NAMED_VARIABLE, std::format("#{}", name)));
        }

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return token(); }
        [[nodiscard]] std::string text() const override { return std::string(token().text()); }

        bool is(const NamedVariableExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "NamedVariableExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        [[nodiscard]] std::string_view name() const { return token().value(); }
    };

    class UnaryExpression final : public RealExpression {
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        using Expression::is;

        enum class Op {
            ADDRESS_OF,
            NEGATIVE,
            POSITIVE
        };

        explicit UnaryExpression(Token token, std::unique_ptr<RealExpression> realExpression) : RealExpression(std::move(token)), m_realExpression(std::move(realExpression)) { }
        ~UnaryExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_realExpression->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("{}{}", token().text(), m_realExpression->text()); }
        [[nodiscard]] const RealExpression *real() const { return m_realExpression.get(); }

        bool is(const UnaryExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "UnaryExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        [[nodiscard]] Op op() const {
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
        using Expression::is;

        enum class Op {
            ASSIGN,
            AND, OR, XOR,
            EQ, NE, LT, LE, GT, GE,
            ADD, SUB,
            MUL, DIV, MOD,
        };

        explicit BinaryExpression(Token token, std::unique_ptr<RealExpression> left, std::unique_ptr<RealExpression> right) : RealExpression(std::move(token)), m_left(std::move(left)), m_right(std::move(right)) { }
        ~BinaryExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return m_left->startToken(); }
        [[nodiscard]] const Token &endToken() const override { return m_right->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("[{} {} {}]", m_left->text(), token().text(), m_right->text()); }

        bool is(const BinaryExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "BinaryExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }

        [[nodiscard]] const RealExpression *left() const { return m_left.get(); }
        [[nodiscard]] const RealExpression *right() const { return m_right.get(); }

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
                default: throw std::logic_error(std::format("invalid token {} '{}' for BinaryExpression", token().name(), token().text()));
            }
        }
    };

    class CallExpression final : public RealExpression {
        Token m_endToken;
        std::vector<std::unique_ptr<ScalarExpression>> m_args;

    public:
        using Expression::is;
        explicit CallExpression(Token token, Token endToken, std::vector<std::unique_ptr<ScalarExpression>> args) : RealExpression(std::move(token)), m_endToken(std::move(endToken)), m_args(std::move(args)) { }
        ~CallExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        [[nodiscard]] std::string text() const override { return std::format("{}[{}]", token().text(), join(m_args, ", ")); }

        bool is(const CallExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "CallExpression"; }
        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }

        [[nodiscard]] std::string_view name() const { return token().value(); }
        [[nodiscard]] const std::vector<std::unique_ptr<ScalarExpression>> &args() const { return m_args; }

        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class GroupingExpression final : public RealExpression {
        Token m_endToken;
        std::unique_ptr<RealExpression> m_realExpression;

    public:
        using Expression::is;
        explicit GroupingExpression(Token token, Token endToken, std::unique_ptr<RealExpression> expression) : RealExpression(std::move(token)), m_endToken(std::move(endToken)), m_realExpression(std::move(expression)) { }
        ~GroupingExpression() override = default;

        [[nodiscard]] const Token &startToken() const override { return token(); }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        [[nodiscard]] std::string text() const override { return std::format("[{}]", m_realExpression->text()); }
        [[nodiscard]] const RealExpression *real() const { return m_realExpression.get(); }

        bool is(const GroupingExpression *) const override { return true; }

        [[nodiscard]] static constexpr const char *staticClassName() { return "GroupingExpression"; }

        [[nodiscard]] constexpr const char *className() const override { return staticClassName(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };
}
