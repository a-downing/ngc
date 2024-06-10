#ifndef BLOCK_H
#define BLOCK_H

#include <optional>
#include <vector>

#include <Utils.h>
#include <Token.h>
#include <Expression.h>
#include <Visitor.h>

namespace ngc
{
    class Statement {
    public:
        virtual ~Statement() = default;
        [[nodiscard]] virtual const Token &startToken() const = 0;
        [[nodiscard]] virtual const Token &endToken() const = 0;
        virtual void accept(Visitor &v, VisitorContext *ctx) const = 0;
        [[nodiscard]] std::string_view text() const { return startToken().source()->text(startToken().start(), endToken().end()); };
    };

    class CompoundStatement final : public Statement {
        std::vector<std::unique_ptr<Statement>> m_statements;

    public:
        explicit CompoundStatement(std::vector<std::unique_ptr<Statement>> statements): m_statements(std::move(statements)) { assert(!m_statements.empty()); }
        ~CompoundStatement() override = default;

        [[nodiscard]] const Token &startToken() const override { return m_statements.front()->startToken(); }
        [[nodiscard]] const Token &endToken() const override { return m_statements.back()->endToken(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
        [[nodiscard]] const std::vector<std::unique_ptr<Statement>> &statements() const { return m_statements; }
    };

    class LineNumber final {
        const Token m_n;
        const Token m_number;

    public:
        LineNumber(const Token &n, const Token &number): m_n(n), m_number(number) { }
        [[nodiscard]] const Token &startToken() const { return m_n; }
        [[nodiscard]] const Token &endToken() const { return m_number; }
        [[nodiscard]] const Token &number() const { return m_number; }
        [[nodiscard]] std::string_view text() const { return startToken().source()->text(startToken().start(), endToken().end()); }
    };

    class BlockStatement final : public Statement {
        std::optional<Token> m_blockDelete;
        std::optional<LineNumber> m_lineNumber;
        std::vector<std::unique_ptr<Expression>> m_expressions;

    public:
        BlockStatement(const std::optional<Token> &blockDelete, const std::optional<LineNumber> &lineNumber, std::vector<std::unique_ptr<Expression>> expressions): m_blockDelete(blockDelete), m_lineNumber(lineNumber), m_expressions(std::move(expressions)) { }
        ~BlockStatement() override = default;

        [[nodiscard]] const std::vector<std::unique_ptr<Expression>> &expressions() const { return m_expressions; }

        [[nodiscard]] const Token &startToken() const override {
            if(m_blockDelete) {
                return *m_blockDelete;
            }

            if(m_lineNumber) {
                return m_lineNumber->startToken();
            }

            assert(!m_expressions.empty());
            return m_expressions.front()->startToken();
        }

        [[nodiscard]] const Token &endToken() const override {
            if(!m_expressions.empty()) {
                return m_expressions.back()->endToken();
            }

            if(m_lineNumber) {
                return m_lineNumber->endToken();
            }

            assert(m_blockDelete);
            return *m_blockDelete;
        }

        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
        [[nodiscard]] const std::optional<Token> &blockDelete() const { return m_blockDelete; }
        [[nodiscard]] const std::optional<LineNumber> &lineNumber() const { return m_lineNumber; }
    };

    class SubStatement final : public Statement {
        Token m_startToken;
        Token m_endToken;
        Token m_identifier;
        std::vector<std::unique_ptr<NamedVariableExpression>> m_params;
        std::unique_ptr<CompoundStatement> m_statements;

    public:
        explicit SubStatement(const Token &startToken, const Token &endToken, const Token &identifier, std::vector<std::unique_ptr<NamedVariableExpression>> params, std::unique_ptr<CompoundStatement> statements): m_startToken(startToken), m_endToken(endToken), m_identifier(identifier), m_params(std::move(params)), m_statements(std::move(statements)) { }
        ~SubStatement() override = default;
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        [[nodiscard]] std::string_view name() const { return m_identifier.value(); }
        [[nodiscard]] const std::vector<std::unique_ptr<NamedVariableExpression>> &params() const { return m_params; } // TODO: maybe this should return a std::vector<NamedVariableExpression *>
        [[nodiscard]] const CompoundStatement *body() const { return m_statements.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class IfStatement final : public Statement {
        Token m_startToken;
        Token m_endToken;
        std::unique_ptr<RealExpression> m_condition;
        std::unique_ptr<CompoundStatement> m_statements;
        std::unique_ptr<CompoundStatement> m_elseStatements;

    public:
        explicit IfStatement(const Token &startToken, const Token &endToken, std::unique_ptr<RealExpression> condition, std::unique_ptr<CompoundStatement> statements, std::unique_ptr<CompoundStatement> elseStatements): m_startToken(startToken), m_endToken(endToken), m_condition(std::move(condition)), m_statements(std::move(statements)), m_elseStatements(std::move(elseStatements)) { }
        ~IfStatement() override = default;
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class WhileStatement final : public Statement {
        Token m_startToken;
        Token m_endToken;
        std::unique_ptr<RealExpression> m_condition;
        std::unique_ptr<CompoundStatement> m_statements;

    public:
        explicit WhileStatement(const Token &startToken, const Token &endToken, std::unique_ptr<RealExpression> condition, std::unique_ptr<CompoundStatement> statements): m_startToken(startToken), m_endToken(endToken), m_condition(std::move(condition)), m_statements(std::move(statements)) { }
        ~WhileStatement() override = default;
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_endToken; }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class ReturnStatement final : public Statement {
        Token m_startToken;
        std::unique_ptr<RealExpression> m_expression;

    public:
        explicit ReturnStatement(const Token &startToken, std::unique_ptr<RealExpression> expression): m_startToken(startToken), m_expression(std::move(expression)) { }
        ~ReturnStatement() override = default;
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_expression->endToken(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };

    class AliasStatement final : public Statement {
        Token m_startToken;
        std::unique_ptr<NamedVariableExpression> m_namedVariable;
        std::unique_ptr<RealExpression> m_expression;

    public:
        explicit AliasStatement(const Token &startToken, std::unique_ptr<NamedVariableExpression> namedVariable, std::unique_ptr<RealExpression> expression): m_startToken(startToken), m_namedVariable(std::move(namedVariable)), m_expression(std::move(expression)) { }
        ~AliasStatement() override = default;
        [[nodiscard]] const Token &startToken() const override { return m_startToken; }
        [[nodiscard]] const Token &endToken() const override { return m_expression->endToken(); }
        [[nodiscard]] const NamedVariableExpression *variable() const { return m_namedVariable.get(); }
        [[nodiscard]] const RealExpression *address() const { return m_expression.get(); }
        void accept(Visitor &v, VisitorContext *ctx) const override { v.visit(this, ctx); }
    };
}

#endif //BLOCK_H
