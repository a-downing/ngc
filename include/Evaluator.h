#ifndef EVALUATOR_H
#define EVALUATOR_H

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

        struct EvaluatorContext : public VisitorContext {
            bool globalScope;
        };

        static const EvaluatorContext &context(VisitorContext *ctx) { return *static_cast<EvaluatorContext *>(ctx); }

    public:
        explicit Evaluator(Memory &mem) : m_mem(mem), m_scope(1), m_subScope(1) { }

        void allocate(const NamedVariableExpression *expr, double value, const EvaluatorContext &ctx) {
            if(ctx.globalScope) {
                if(m_scope.front().contains(expr->name())) {
                    throw std::logic_error(std::format("redeclared global variable '{}'", expr->name()));
                }

                uint32_t addr = m_mem.addData(MemoryCell(MemoryCell::Flags::READ | MemoryCell::Flags::WRITE, value));
                std::println("{}: ALLOCATE GLOBAL: {} @ data:{}", expr->token().location(), expr->text(), addr);
                m_scope.front().emplace(expr->name(), addr);
            } else {
                if(m_scope.back().contains(expr->name())) {
                    throw std::logic_error(std::format("redeclared local variable '{}'", expr->name()));
                }

                uint32_t addr = m_mem.push(value);
                std::println("{}: ALLOCATE LOCAL: {} @ stack:{}", expr->token().location(), expr->text(), addr & ~Memory::ADDR_STACK);
                m_scope.back().emplace(expr->name(), addr);
            }
        }

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

        void executeProgram(const CompoundStatement *stmt) {
            if(stmt) {
                auto ctx = EvaluatorContext { .globalScope = true };
                stmt->accept(*this, &ctx);
            }
        }

        void visit(const CompoundStatement* stmt, VisitorContext* ctx) override { 
            for(const auto &s : stmt->statements()) {
                s->accept(*this, ctx);
            }
        }

        void visit(const AliasStatement *stmt, VisitorContext *ctx) override {
            auto addr = static_cast<uint32_t>(eval(stmt->address()));

            if(context(ctx).globalScope) {
                m_scope.front().emplace(stmt->variable()->name(), addr);
            } else {
                m_scope.back().emplace(stmt->variable()->name(), addr);
            }
        }

        void visit(const LetStatement* stmt, VisitorContext* ctx) override {
            double value = 0.0;

            if(stmt->value()) {
                value = eval(stmt->value());
            }

            allocate(stmt->variable(), value, context(ctx));
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
            auto value = eval(stmt->real());
            std::println("execute: ReturnStatement: {} -> {}", stmt->real()->text(), value);
            std::ignore = m_mem.push(value);
        }

        void visit(const CallExpression* expr, VisitorContext* ctx) override {
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

            m_scope.emplace_back();
            m_subScope.emplace_back();

            auto newCtx = EvaluatorContext { .globalScope = false };

            for(size_t i = 0; i < params.size(); i++) {
                const auto value = eval(args[i].get());
                allocate(params[i].get(), value, newCtx);
            }

            if(stmt->body()) {
                stmt->body()->accept(*this, &newCtx);
            }

            // return value
            m_accumulator = m_mem.pop();
            std::println("{}: execute: {}: {} -> {}", expr->token().location(), expr->className(), expr->text(), m_accumulator);

            for(size_t i = 0; i < m_scope.back().size() - 1; i++) {
                std::ignore = m_mem.pop();
            }

            m_scope.pop_back();
            m_subScope.pop_back();
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
                case BinaryExpression::Op::ASSIGN: m_accumulator = right; break;
                case BinaryExpression::Op::AND: m_accumulator = static_cast<bool>(left) && static_cast<bool>(right); break;
                case BinaryExpression::Op::OR: m_accumulator = static_cast<bool>(left) || static_cast<bool>(right); break;
                case BinaryExpression::Op::XOR: m_accumulator = (static_cast<bool>(left) && !static_cast<bool>(right)) || (static_cast<bool>(right) && !static_cast<bool>(left)); break;
                case BinaryExpression::Op::EQ: m_accumulator = left == right; break;
                case BinaryExpression::Op::NE: m_accumulator = left != right; break;
                case BinaryExpression::Op::LT: m_accumulator = left < right; break;
                case BinaryExpression::Op::LE: m_accumulator = left <= right; break;
                case BinaryExpression::Op::GT: m_accumulator = left > right; break;
                case BinaryExpression::Op::GE: m_accumulator = left >= right; break;
                case BinaryExpression::Op::ADD: m_accumulator = left + right; break;
                case BinaryExpression::Op::SUB: m_accumulator = left - right; break;
                case BinaryExpression::Op::MUL: m_accumulator = left * right; break;
                case BinaryExpression::Op::DIV: m_accumulator = left / right; break;
                case BinaryExpression::Op::MOD: m_accumulator = std::fmod(left, right); break;
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
        
        // TODO: these
        void visit(const IfStatement* stmt, VisitorContext* ctx) override { throw std::logic_error(std::format("tried to evaluate IfStatement")); }
        void visit(const WhileStatement* stmt, VisitorContext* ctx) override { throw std::logic_error(std::format("tried to evaluate WhileStatement")); }
        
        // not implemented for now
        void visit(const CommentExpression* expr, VisitorContext* ctx) override { }
        void visit(const WordExpression* expr, VisitorContext* ctx) override { }

        double read(const uint32_t addr) {
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
    };
}

#endif //EVALUATOR_H
