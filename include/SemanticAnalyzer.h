
#ifndef SEMANTICANALYZER_H
#define SEMANTICANALYZER_H

#include <print>
#include <ranges>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include <Statement.h>
#include <Expression.h>
#include <VisitorContext.h>

#include "Evaluator.h"

namespace ngc
{
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

namespace ngc
{
    class SemanticAnalyzer final : public Visitor {
    public:
        class Error {
            std::string m_message;
            const Statement *m_statement = nullptr;
            const Expression *m_expression = nullptr;

        public:
            Error(std::string message, const Statement *stmt) : m_message(std::move(message)), m_statement(stmt) { }
            Error(std::string message, const Expression *expr) : m_message(std::move(message)), m_expression(expr) { }
            [[nodiscard]] const std::string &message() const { return m_message; }
        };

    private:
        std::vector<std::unordered_map<std::string_view, const NamedVariableExpression *>> m_scope;
        std::vector<std::unordered_set<SubSignature>> m_subScope;
        std::vector<Error> m_errors;
        bool m_declareVariables = false;

        class SemanticAnalyzerContext final : public VisitorContext {
            bool m_globalScope;

        public:
            explicit SemanticAnalyzerContext(const bool globalScope) : m_globalScope(globalScope) { }
            [[nodiscard]] bool isGlogalScope() const { return m_globalScope; }
            [[nodiscard]] bool isLocalScope() const { return !m_globalScope; }
        };

        static const SemanticAnalyzerContext &context(VisitorContext *ctx) { return *static_cast<SemanticAnalyzerContext *>(ctx); }

    public:
        SemanticAnalyzer() : m_scope(1), m_subScope(1) { }
        ~SemanticAnalyzer() override = default;

        void reset() {
            m_scope.clear();
            m_scope.resize(1);
            m_subScope.clear();
            m_subScope.resize(1);
            m_errors.clear();
        }

        [[nodiscard]] const std::vector<Error> &errors() const { return m_errors; }
        void clearErrors() { m_errors.clear(); }

        void processPreamble(const Preamble &preamble) {
            if(preamble.statements()) {
                auto ctx = SemanticAnalyzerContext(true);
                preamble.statements()->accept(*this, &ctx);
            }
        }

        void processProgram(const CompoundStatement *program, const bool declareVariables) {
            m_declareVariables = declareVariables;

            if(program) {
                auto ctx = SemanticAnalyzerContext(true);
                program->accept(*this, &ctx);
            }
        }

        void addGlobalSub(const SubSignature &s) {
            m_subScope.front().emplace(s);
        }

        bool isDeclared(const NamedVariableExpression *expr) const {
            return std::ranges::any_of(m_scope, [&expr](const auto &scope) {
                return scope.contains(expr->name());
            });
        }

        bool isDeclared(const CallExpression *expr) const {
            return std::ranges::any_of(std::views::reverse(m_subScope), [&expr](const auto &scope) {
                return scope.contains(SubSignature(expr));
            });
        }

        void declareLocal(const NamedVariableExpression *expr) {
            if(!m_declareVariables) {
                return;
            }

            if(!m_scope.back().contains(expr->name())) {
                m_scope.back().emplace(expr->name(), expr);
                std::println("{}: INFO: declared local variable '{}'", expr->startToken().location(), expr->name());
                return;
            }

            m_errors.emplace_back(std::format("{}: ERROR: local variable '{}' already declared", expr->startToken().location(), expr->name()), expr);
        }

        void declareGlobal(const NamedVariableExpression *expr, const SemanticAnalyzerContext &ctx) {
            if(!m_declareVariables) {
                return;
            }

            if(!m_scope.front().contains(expr->name())) {
                m_scope.front().emplace(expr->name(), expr);
                std::println("{}: INFO: declared global variable '{}'", expr->startToken().location(), expr->name());
                return;
            }

            m_errors.emplace_back(std::format("{}: ERROR: global variable '{}' already declared", expr->startToken().location(), expr->name()), expr);
        }

        void declareSub(const SubStatement *stmt) {
            if(!m_declareVariables) {
                return;
            }

            if(m_subScope.back().contains(SubSignature(stmt))) {
                m_errors.emplace_back(std::format("{}: ERROR: redefinition of sub '{}'", stmt->startToken().location(), stmt->name()), stmt);
            } else {
                m_subScope.back().emplace(stmt);
            }

            m_scope.emplace_back();
        }

        // statements
        void visit(const CompoundStatement *stmt, VisitorContext *ctx) override {
            for(const auto &s : stmt->statements()) {
                s->accept(*this, ctx);
            }
        }

        void visit(const BlockStatement *stmt, VisitorContext *ctx) override {
            for(const auto &expr : stmt->expressions()) {
                expr->accept(*this, ctx);
            }
        }

        void visit(const SubStatement *stmt, VisitorContext *ctx) override {
            declareSub(stmt);

            m_scope.emplace_back();

            for(const auto &param : stmt->params()) {
                declareLocal(param.get());
            }

            if(stmt->body()) {
                auto localCtx = SemanticAnalyzerContext(false);

                for(const auto &s : stmt->body()->statements()) {
                    s->accept(*this, &localCtx);
                }
            }

            m_scope.pop_back();
        }

        void visit(const AliasStatement *stmt, VisitorContext *ctx) override {
            if(context(ctx).isGlogalScope()) {
                declareGlobal(stmt->variable(), context(ctx));
            } else {
                declareLocal(stmt->variable());
            }

            stmt->address()->accept(*this, ctx);
        }

        void visit(const LetStatement *stmt, VisitorContext *ctx) override {
            if(context(ctx).isGlogalScope()) {
                declareGlobal(stmt->variable(), context(ctx));
            } else {
                declareLocal(stmt->variable());
            }

            if(stmt->value()) {
                stmt->value()->accept(*this, ctx);
            }
        }

        void visit(const IfStatement *stmt, VisitorContext *ctx) override {
            stmt->condition()->accept(*this, ctx);

            if(stmt->body()) {
                stmt->body()->accept(*this, ctx);
            }

            if(stmt->elseBody()) {
                stmt->elseBody()->accept(*this, ctx);
            }
        }

        void visit(const WhileStatement *stmt, VisitorContext *ctx) override {
            stmt->condition()->accept(*this, ctx);

            if(stmt->body()) {
                stmt->body()->accept(*this, ctx);
            }
        }

        void visit(const ReturnStatement *stmt, VisitorContext *ctx) override {
            stmt->real()->accept(*this, ctx);
        }

        // expressions
        void visit(const BinaryExpression *expr, VisitorContext *ctx) override {
            expr->left()->accept(*this, ctx);
            expr->right()->accept(*this, ctx);
        }

        void visit(const NamedVariableExpression *expr, VisitorContext *ctx) override {
            if(!isDeclared(expr)) {
                m_errors.emplace_back(std::format("{}: ERROR: variable '{}' accessed before assignment", expr->token().location(), expr->name()), expr);
            }
        }

        void visit(const CallExpression *expr, VisitorContext *ctx) override {
            if(!isDeclared(expr)) {
                m_errors.emplace_back(std::format("{}: ERROR: call to '{}' before definition", expr->token().location(), expr->toString()), expr);
            }
        }

        void visit(const WordExpression *expr, VisitorContext *ctx) override {
            expr->real()->accept(*this, ctx);
        }

        void visit(const UnaryExpression *expr, VisitorContext *ctx) override {
            expr->real()->accept(*this, ctx);
        }

        void visit(const GroupingExpression *expr, VisitorContext *ctx) override {
            expr->real()->accept(*this, ctx);
        }

        void visit(const NumericVariableExpression *expr, VisitorContext *ctx) override {
            expr->real()->accept(*this, ctx);
        }

        // not needed
        void visit(const CommentExpression *expr, VisitorContext *ctx) override { }
        void visit(const LiteralExpression *expr, VisitorContext *ctx) override { }
    };
}

#endif //SEMANTICANALYZER_H
