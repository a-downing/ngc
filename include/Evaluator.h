#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <print>
#include <cmath>
#include <cstddef>
#include <format>
#include <stdexcept>
#include <unordered_map>
#include <ranges>

#include <Visitor.h>
#include <VisitorContext.h>
#include <Expression.h>
#include <Statement.h>
#include <MemoryCell.h>
#include <Memory.h>
#include <SubSignature.h>

namespace ngc
{
    class Evaluator final : public Visitor {
        std::vector<std::unordered_map<std::string_view, uint32_t>> m_scope;
        std::vector<std::unordered_map<SubSignature, const SubStatement *>> m_subScope;
        double m_accumulator = 0.0;
        Memory &m_mem;

        class Scope {
            Evaluator *m_evaluator = nullptr;
            bool m_opened = false;
            bool m_global = false;

        public:
            explicit Scope(Evaluator &evaluator, const bool global) : m_evaluator(&evaluator), m_global(global) { }
            Scope() = default;

            ~Scope() {
                if(m_opened) {
                    for(auto _ : m_evaluator->m_scope.back()) {
                        m_evaluator->m_mem.pop();
                    }

                    m_evaluator->m_scope.pop_back();
                    m_evaluator->m_subScope.pop_back();
                }
            }

            void allocate(const NamedVariableExpression *expr, double value) {
                std::println("{}: allocate: {} {}", expr->token().location(), expr->text(), value);

                if(!m_global && !m_opened) {
                    openScope();
                }

                if(m_evaluator->m_scope.size() == 1) {
                    if(m_evaluator->m_scope.front().contains(expr->name())) {
                        throw std::logic_error(std::format("redeclared global variable '{}'", expr->name()));
                    }

                    uint32_t addr = m_evaluator->m_mem.addData(MemoryCell(MemoryCell::Flags::READ | MemoryCell::Flags::WRITE, value));
                    std::println("{}: ALLOCATE GLOBAL: {} @ data:{}", expr->token().location(), expr->text(), addr);
                    m_evaluator->m_scope.front().emplace(expr->name(), addr);
                } else {
                    if(m_evaluator->m_scope.back().contains(expr->name())) {
                        throw std::logic_error(std::format("redeclared local variable '{}'", expr->name()));
                    }

                    uint32_t addr = m_evaluator->m_mem.push(value);
                    std::println("{}: ALLOCATE LOCAL: {} @ stack:{}", expr->token().location(), expr->text(), addr & ~Memory::ADDR_STACK);
                    m_evaluator->m_scope.back().emplace(expr->name(), addr);
                }
            }

        private:
            void openScope() {
                m_evaluator->m_scope.emplace_back();
                m_evaluator->m_subScope.emplace_back();
                m_opened = true;
            }
        };

        struct Context final : VisitorContext {
            enum Action {
                NONE,
                RETURN,
                BREAK,
                CONTINUE
            };

            Scope scope;
            Action action = NONE;
        };

        static Context *context(VisitorContext *ctx) { return static_cast<Context *>(ctx); }

    public:
        explicit Evaluator(Memory &mem) : m_scope(1), m_subScope(1), m_mem(mem) { }

        Context createScopeContext(bool global = false) { return Context { .scope = Scope(*this, global) }; }

        void declareSub(const SubStatement *stmt) {
            auto sig = SubSignature(stmt);

            if(m_subScope.back().contains(sig)) {
                throw std::logic_error(std::format("redeclared subroutine '{}'", sig.name()));
            }

            m_subScope.back().emplace(sig, stmt);
        }

        double eval(const RealExpression *expr, const bool dereference = true) {
            expr->accept(*this, nullptr);

            if(dereference && expr->is<VariableExpression>()) {
                const auto addr = static_cast<uint32_t>(m_accumulator);
                return read(addr);
            }

            return m_accumulator;
        }

        void executeProgram(const std::vector<std::unique_ptr<Statement>> &program) {
            auto ctx = createScopeContext(true);

            for(const auto &stmt : program) {
                stmt->accept(*this, &ctx);
            }
        }

        void visit(const ExpressionStatement* stmt, VisitorContext* ctx) override {
            stmt->expression()->accept(*this, ctx);
        }

        void visit(const CompoundStatement* stmt, VisitorContext* ctx) override {
            Context newCtx;

            if(!ctx) {
                newCtx = createScopeContext();
                ctx = &newCtx;
            }

            for(const auto &s : stmt->statements()) {
                s->accept(*this, ctx);
            }
        }

        void visit(const AliasStatement *stmt, VisitorContext *ctx) override {
            auto addr = static_cast<uint32_t>(eval(stmt->address()));
            m_scope.back().emplace(stmt->variable()->name(), addr);
        }

        void visit(const LetStatement* stmt, VisitorContext* ctx) override {
            double value = 0.0;

            if(stmt->value()) {
                value = eval(stmt->value());
            }

            context(ctx)->scope.allocate(stmt->variable(), value);
        }

        void visit(const SubStatement* stmt, VisitorContext* ctx) override { 
            declareSub(stmt);
        }

        void visit(const BlockStatement* stmt, VisitorContext* ctx) override {
            for(const auto &expr : stmt->expressions()) {
                expr->accept(*this, ctx);
            }
        }

        void visit(const ReturnStatement* stmt, VisitorContext* ctx) override {
            m_accumulator = eval(stmt->real());
            context(ctx)->action = Context::RETURN;
        }

        void visit(const BreakStatement* stmt, VisitorContext* ctx) override {
            context(ctx)->action = Context::BREAK;
        }

        void visit(const ContinueStatement* stmt, VisitorContext* ctx) override {
            context(ctx)->action = Context::CONTINUE;
        }

        void visit(const IfStatement* stmt, VisitorContext* ctx) override {
            if(eval(stmt->condition()) != 0.0) {
                std::println("<IF TRUE>");
                auto newCtx = createScopeContext();
                stmt->body()->accept(*this, &newCtx);
                context(ctx)->action = newCtx.action;
            } else {
                std::println("<IF FALSE>");
                if(stmt->elseBody()) {
                    auto newCtx = createScopeContext();
                    stmt->elseBody()->accept(*this, &newCtx);
                    context(ctx)->action = newCtx.action;
                }
            }
        }

        void visit(const WhileStatement* stmt, VisitorContext* ctx) override {
            Context newCtx = createScopeContext();

            while(eval(stmt->condition()) != 0.0) {
                newCtx.action = Context::NONE;
                bool doBreak = false;

                for(const auto &s : stmt->body()->statements()) {
                    s->accept(*this, &newCtx);

                    if(newCtx.action == Context::CONTINUE) {
                        std::println("<CONTINUE>");
                        break;
                    }

                    if(newCtx.action == Context::BREAK) {
                        std::println("<BREAK>");
                        doBreak = true;
                        break;
                    }
                }

                if(doBreak) {
                    break;
                }
            }
        }

        void visit(const CallExpression* expr, VisitorContext* ctx) override {
            // functions implicitly return 0 if they dont return explicitly
            m_accumulator = 0.0;

            // TODO: more extensible way to evaluate built in functions
            if(expr->name() == "print") {
                std::string text;

                for(const auto &arg : expr->args()) {
                    if(const auto real = arg->as<RealExpression>()) {
                        text += std::format("{}", eval(real));
                    } else if(const auto str = arg->as<StringExpression>()) {
                        text += str->value();
                    }
                }

                std::println("<PRINT>: {}", text);
                return;
            }

            const auto sig = SubSignature(expr);
            const SubStatement *stmt = nullptr;

            for(const auto &scope : std::views::reverse(m_subScope)) {
                if(scope.contains(sig)) {
                    stmt = scope.at(sig);
                    break;
                }
            }

            if(stmt == nullptr) {
                throw std::logic_error(std::format("undefined sub '{}'", sig.toString()));
            }

            auto &params = stmt->params();
            auto &args = expr->args();

            if(params.size() != args.size()) {
                throw std::runtime_error("params.size() != args.size()");
            }

            Context newCtx = createScopeContext();

            for(size_t i = 0; i < params.size(); i++) {
                const auto arg = expect<RealExpression>(args[i].get());
                const auto value = eval(arg);
                newCtx.scope.allocate(params[i].get(), value);
            }

            for(const auto &s : stmt->body()->statements()) {
                stmt->body()->accept(*this, &newCtx);

                if(newCtx.action == Context::RETURN) {
                    break;
                }
            }


            std::println("{}: execute: {}: {} -> {}", expr->token().location(), expr->className(), expr->text(), m_accumulator);
        }

        void visit(const NumericVariableExpression* expr, VisitorContext* ctx) override {
            const auto addr = static_cast<uint32_t>(eval(expr->real()));
            m_accumulator = addr;
            std::println("{}: execute: {}: {} -> {}", expr->token().location(), expr->className(), expr->text(), m_accumulator);
        }

        void visit(const NamedVariableExpression* expr, VisitorContext* ctx) override {
            uint32_t addr = 0;

            for(const auto &scope : std::views::reverse(m_scope)) {
                if(scope.contains(expr->name())) {
                    addr = scope.at(expr->name());
                    break;
                }
            }

            if(addr == 0) {
                throw std::runtime_error(std::format("undeclared variable '{}'", expr->name()));
            }

            if(addr & Memory::ADDR_STACK) {
                std::println("{}: execute: {}: {} -> stack:{}", expr->token().location(), expr->className(), expr->text(), addr & ~Memory::ADDR_STACK);
            } else {
                std::println("{}: execute: {}: {} -> data:{}", expr->token().location(), expr->className(), expr->text(), addr);
            }

            m_accumulator = addr;
        }

        void visit(const BinaryExpression* expr, VisitorContext* ctx) override {
            double left;
            double right;

            if(expr->op() == BinaryExpression::Op::ASSIGN) {
                if(!expr->left()->is<VariableExpression>()) {
                    throw std::runtime_error(std::format("tried to assign to {}", expr->className()));
                }

                left = eval(expr->left(), false);
                right = eval(expr->right());
                const auto addr = static_cast<uint32_t>(left);
                write(addr, right);
            } else {
                left = eval(expr->left());
                right = eval(expr->right());
            }

            switch (expr->op()) {
                using enum BinaryExpression::Op;
                case ASSIGN: m_accumulator = right; break;
                case AND: m_accumulator = static_cast<bool>(left) && static_cast<bool>(right); break;
                case OR: m_accumulator = static_cast<bool>(left) || static_cast<bool>(right); break;
                case XOR: m_accumulator = (static_cast<bool>(left) && !static_cast<bool>(right)) || (static_cast<bool>(right) && !static_cast<bool>(left)); break;
                case EQ: m_accumulator = left == right; break;
                case NE: m_accumulator = left != right; break;
                case LT: m_accumulator = left < right; break;
                case LE: m_accumulator = left <= right; break;
                case GT: m_accumulator = left > right; break;
                case GE: m_accumulator = left >= right; break;
                case ADD: m_accumulator = left + right; break;
                case SUB: m_accumulator = left - right; break;
                case MUL: m_accumulator = left * right; break;
                case DIV: m_accumulator = left / right; break;
                case MOD: m_accumulator = std::fmod(left, right); break;
            }

            std::println("{}: execute: {}: {} -> {}", expr->token().location(), expr->className(), expr->text(), m_accumulator);
        }

        void visit(const UnaryExpression* expr, VisitorContext* ctx) override {
            double value;

            if(expr->op() == UnaryExpression::Op::ADDRESS_OF) {
                if(!expr->real()->is<VariableExpression>()) {
                    throw std::runtime_error(std::format("tried to assign to {}", expr->className()));
                }

                value = static_cast<uint32_t>(eval(expr->real(), false));
            } else {
                value = eval(expr->real());
            }

            switch (expr->op()) {
                case UnaryExpression::Op::NEGATIVE: m_accumulator = -value; break;
                case UnaryExpression::Op::POSITIVE: m_accumulator = value; break;
                case UnaryExpression::Op::ADDRESS_OF: m_accumulator = value; break;
            }

            std::println("{}: execute: {}: {} -> {}", expr->token().location(), expr->className(), expr->text(), m_accumulator);
        }

        void visit(const GroupingExpression* expr, VisitorContext* ctx) override {
            m_accumulator = eval(expr->real());
            std::println("{}: execute: {}: {} -> {}", expr->token().location(), expr->className(), expr->text(), m_accumulator);
        }

        void visit(const LiteralExpression* expr, VisitorContext* ctx) override {
            m_accumulator = expr->value();
            std::println("{}: execute: {}: {} -> {}", expr->token().location(), expr->className(), expr->text(), m_accumulator);
        }
        
        // not implemented for now
        void visit(const CommentExpression* expr, VisitorContext* ctx) override { }
        void visit(const WordExpression* expr, VisitorContext* ctx) override { }
        void visit(const StringExpression* expr, VisitorContext* ctx) override { }

        [[nodiscard]] double read(const uint32_t addr) const {
            auto result = m_mem.read(addr);

            if(!result) {
                switch (result.error()) {
                case Memory::Error::INVALID_DATA_ADDRESS: throw std::logic_error("INVALID_DATA_ADDRESS");
                case Memory::Error::INVALID_STACK_ADDRESS: throw std::logic_error("INVALID_STACK_ADDRESS");
                case Memory::Error::READ: throw std::logic_error("READ");
                case Memory::Error::WRITE: throw std::logic_error("WRITE");
                }
            }

            return *result;
        }

        void write(const uint32_t addr, const double value) {
            auto result = m_mem.write(addr, value);

            if(!result) {
                switch (result.error()) {
                case Memory::Error::INVALID_DATA_ADDRESS: throw std::logic_error("INVALID_DATA_ADDRESS");
                case Memory::Error::INVALID_STACK_ADDRESS: throw std::logic_error("INVALID_STACK_ADDRESS");
                case Memory::Error::READ: throw std::logic_error("READ");
                case Memory::Error::WRITE: throw std::logic_error("WRITE");
                }
            }
        }

        template<typename T>
        const T *expect(const Expression *expr) {
            auto t = expr->as<T>();

            if(!t) {
                throw std::logic_error(std::format("expected {}, but found {}", T::staticClassName(), expr->className()));
            }

            return t;
        }
    };
}

#endif //EVALUATOR_H
