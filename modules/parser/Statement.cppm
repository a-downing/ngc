module;

#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

export module parser:Statement;
import :Token;
import :Expression;
import :Visitor;

export namespace ngc
{
    class Statement {
    public:
        virtual ~Statement() = default;
        [[nodiscard]] virtual const Token &startToken() const = 0;
        [[nodiscard]] virtual const Token &endToken() const = 0;

        virtual std::string text() const = 0;

        virtual bool is(const class ExpressionStatement *) const { return false; }
        virtual bool is(const class CompoundStatement *) const { return false; }
        virtual bool is(const class BlockStatement *) const { return false; }
        virtual bool is(const class SubStatement *) const { return false; }
        virtual bool is(const class IfStatement *) const { return false; }
        virtual bool is(const class WhileStatement *) const { return false; }
        virtual bool is(const class ReturnStatement *) const { return false; }
        virtual bool is(const class BreakStatement *) const { return false; }
        virtual bool is(const class ContinueStatement *) const { return false; }
        virtual bool is(const class AliasStatement *) const { return false; }
        virtual bool is(const class LetStatement *) const { return false; }

        template<typename T>
        [[nodiscard]] bool is() const {
            return this->is(static_cast<const T *>(this));
        }

        template<typename T>
        const T *as() const {
            return this->is(static_cast<const T *>(this)) ? static_cast<const T *>(this) : nullptr;
        }

        template<typename T>
        T *as() {
            return this->is(static_cast<T *>(this)) ? static_cast<T *>(this) : nullptr;
        }

        virtual void accept(Visitor &v, VisitorContext *ctx) const = 0;
    };

    class ExpressionStatement final : public Statement {
        std::unique_ptr<Expression> m_expression;

    public:
        explicit ExpressionStatement(std::unique_ptr<Expression> expression): m_expression(std::move(expression)) { }
        ~ExpressionStatement() override = default;
        bool is(const ExpressionStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_expression->startToken(); }
        [[nodiscard]] const Token &endToken() const override { return m_expression->endToken(); }
        [[nodiscard]] std::string text() const override { return m_expression->text(); }
        const Expression *expression() const { return m_expression.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class CompoundStatement final : public Statement {
        Token m_startToken;
        Token m_endToken;
        std::vector<std::unique_ptr<Statement>> m_statements;

    public:
        CompoundStatement(Token startToken, Token endToken, std::vector<std::unique_ptr<Statement>> statements): m_startToken(std::move(startToken)), m_endToken(std::move(endToken)), m_statements(std::move(statements)) { }
        ~CompoundStatement() override = default;
        bool is(const CompoundStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        [[nodiscard]] std::string text() const override { return std::format("{{ {} }}", join(m_statements, " ")); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
        [[nodiscard]] const std::vector<std::unique_ptr<Statement>> &statements() const { return m_statements; }
        [[nodiscard]] std::vector<std::unique_ptr<Statement>> &statements() { return m_statements; }
    };

    class BlockStatement final : public Statement {
        std::optional<Token> m_blockDelete;
        std::vector<std::unique_ptr<WordExpression>> m_expressions;

    public:
        BlockStatement(std::optional<Token> blockDelete, std::vector<std::unique_ptr<WordExpression>> expressions): m_blockDelete(std::move(blockDelete)), m_expressions(std::move(expressions)) { }
        ~BlockStatement() override = default;
        bool is(const BlockStatement *) const override { return true; }
        [[nodiscard]] const std::vector<std::unique_ptr<WordExpression>> &expressions() const { return m_expressions; }

        [[nodiscard]] const Token &startToken() const override {
            if(m_blockDelete) {
                return *m_blockDelete;
            }

            if(m_expressions.empty()) {
                throw std::runtime_error("empty BlockStatement");
            }
            
            return m_expressions.front()->startToken();
        }

        [[nodiscard]] const Token &endToken() const override {
            if(!m_expressions.empty()) {
                return m_expressions.back()->endToken();
            }

            if(!m_blockDelete) {
                throw std::runtime_error("empty BlockStatement");
            }

            return *m_blockDelete;
        }

        [[nodiscard]] std::string text() const override { return std::format("{}{}", m_blockDelete ? m_blockDelete->text() : "", join(m_expressions, " ")); }

        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
        [[nodiscard]] const std::optional<Token> &blockDelete() const { return m_blockDelete; }
    };

    class SubStatement final : public Statement {
        Token m_startToken;
        Token m_identifier;
        std::vector<std::unique_ptr<NamedVariableExpression>> m_params;
        std::unique_ptr<CompoundStatement> m_statement;

    public:
        explicit SubStatement(Token startToken, Token identifier, std::vector<std::unique_ptr<NamedVariableExpression>> params, std::unique_ptr<CompoundStatement> statements): m_startToken(std::move(startToken)), m_identifier(std::move(identifier)), m_params(std::move(params)), m_statement(std::move(statements)) { }
        ~SubStatement() override = default;
        bool is(const SubStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_statement->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("{}[{}] {}", name(), join(m_params, ", "), m_statement->text()); }
        [[nodiscard]] std::string_view name() const { return m_identifier.value(); }
        [[nodiscard]] const std::vector<std::unique_ptr<NamedVariableExpression>> &params() const { return m_params; } // TODO: maybe this should return a std::vector<NamedVariableExpression *>
        [[nodiscard]] const CompoundStatement *body() const { return m_statement.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class IfStatement final : public Statement {
        Token m_startToken;
        std::unique_ptr<RealExpression> m_condition;
        std::unique_ptr<CompoundStatement> m_statements;
        std::unique_ptr<CompoundStatement> m_elseStatements;

    public:
        explicit IfStatement(Token startToken, std::unique_ptr<RealExpression> condition, std::unique_ptr<CompoundStatement> statements, std::unique_ptr<CompoundStatement> elseStatements): m_startToken(std::move(startToken)), m_condition(std::move(condition)), m_statements(std::move(statements)), m_elseStatements(std::move(elseStatements)) { }
        ~IfStatement() override = default;
        bool is(const IfStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_elseStatements ? m_elseStatements->endToken() : m_statements->endToken(); }

        [[nodiscard]] std::string text() const override { 
            if(m_elseStatements) {
                return std::format("{} {} {} else {}", m_startToken.text(), m_condition->text(), m_statements->text(), m_elseStatements->text());
            }

            return std::format("{} {} {}", m_startToken.text(), m_condition->text(), m_statements->text());
        }
            

        [[nodiscard]] const RealExpression *condition() const { return m_condition.get(); }
        [[nodiscard]] const CompoundStatement *body() const { return m_statements.get(); }
        [[nodiscard]] const CompoundStatement *elseBody() const { return m_elseStatements.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class WhileStatement final : public Statement {
        Token m_startToken;
        std::unique_ptr<RealExpression> m_condition;
        std::unique_ptr<CompoundStatement> m_statements;

    public:
        explicit WhileStatement(Token startToken, std::unique_ptr<RealExpression> condition, std::unique_ptr<CompoundStatement> statements): m_startToken(std::move(startToken)), m_condition(std::move(condition)), m_statements(std::move(statements)) { }
        ~WhileStatement() override = default;
        bool is(const WhileStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_statements->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("{} {} {}", m_startToken.text(), m_condition->text(), m_statements->text()); }
        [[nodiscard]] const RealExpression *condition() const { return m_condition.get(); }
        [[nodiscard]] const CompoundStatement *body() const { return m_statements.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class ReturnStatement final : public Statement {
        Token m_startToken;
        std::unique_ptr<RealExpression> m_expression;

    public:
        explicit ReturnStatement(Token startToken, std::unique_ptr<RealExpression> expression): m_startToken(std::move(startToken)), m_expression(std::move(expression)) { }
        ~ReturnStatement() override = default;
        bool is(const ReturnStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_expression->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("{} {}", m_startToken.text(), m_expression->text()); }
        [[nodiscard]] const RealExpression *real() const { return m_expression.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class BreakStatement final : public Statement {
        Token m_startToken;

    public:
        explicit BreakStatement(Token startToken): m_startToken(std::move(startToken)) { }
        ~BreakStatement() override = default;
        bool is(const BreakStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_startToken; }
        [[nodiscard]] std::string text() const override { return std::string(m_startToken.text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class ContinueStatement final : public Statement {
        Token m_startToken;

    public:
        explicit ContinueStatement(Token startToken): m_startToken(std::move(startToken)) { }
        ~ContinueStatement() override = default;
        bool is(const ContinueStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_startToken; }
        [[nodiscard]] std::string text() const override { return std::string(m_startToken.text()); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class AliasStatement final : public Statement {
        Token m_startToken;
        std::unique_ptr<NamedVariableExpression> m_namedVariable;
        std::unique_ptr<RealExpression> m_expression;

    public:
        explicit AliasStatement(Token startToken, std::unique_ptr<NamedVariableExpression> namedVariable, std::unique_ptr<RealExpression> expression): m_startToken(std::move(startToken)), m_namedVariable(std::move(namedVariable)), m_expression(std::move(expression)) { }
        ~AliasStatement() override = default;
        bool is(const AliasStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_expression->endToken(); }
        [[nodiscard]] std::string text() const override { return std::format("{} {} = {}", m_startToken.text(), m_namedVariable->text(), m_expression->text()); }
        [[nodiscard]] const NamedVariableExpression *variable() const { return m_namedVariable.get(); }
        [[nodiscard]] const RealExpression *address() const { return m_expression.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class LetStatement final : public Statement {
        Token m_startToken;
        std::unique_ptr<NamedVariableExpression> m_namedVariable;
        std::unique_ptr<RealExpression> m_expression;

    public:
        explicit LetStatement(Token startToken, std::unique_ptr<NamedVariableExpression> namedVariable, std::unique_ptr<RealExpression> expression): m_startToken(std::move(startToken)), m_namedVariable(std::move(namedVariable)), m_expression(std::move(expression)) { }
        ~LetStatement() override = default;
        bool is(const LetStatement *) const override { return true; }
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_expression->endToken(); }

        [[nodiscard]] std::string text() const override { 
            if(m_expression) {
                return std::format("{} {} = {}", m_startToken.text(), m_namedVariable->text(), m_expression->text());
            }

            return std::format("{} {}", m_startToken.text(), m_namedVariable->text());
        }

        [[nodiscard]] const NamedVariableExpression *variable() const { return m_namedVariable.get(); }
        [[nodiscard]] const RealExpression *value() const { return m_expression.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };
}
