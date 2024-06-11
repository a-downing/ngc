#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <format>

#include <Utils.h>
#include <Visitor.h>
#include <Expression.h>
#include <Memory.h>

namespace ngc
{
    class Evaluator final : public Visitor {
        double m_accumulator = 0.0;
        Memory &m_mem;

    public:
        explicit Evaluator(Memory &mem) : m_mem(mem) { }

        double eval(const RealExpression *expr) {
            expr->accept(*this, nullptr);
            return m_accumulator;
        }

        void visit(const BinaryExpression* expr, VisitorContext* ctx) override {
            expr->left()->accept(*this, nullptr);
            const auto left = m_accumulator;

            expr->right()->accept(*this, nullptr);
            const auto right = m_accumulator;

            switch (expr->op()) {
            case BinaryExpression::Op::ASSIGN: m_accumulator = right; break;
            case BinaryExpression::Op::AND: m_accumulator = left && right; break;
            case BinaryExpression::Op::OR: m_accumulator = left || right; break;
            case BinaryExpression::Op::XOR: m_accumulator = (left && !right) || (right && !left); break;
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
        }

        void visit(const UnaryExpression* expr, VisitorContext* ctx) override {
            expr->real()->accept(*this, nullptr);
            const auto value = m_accumulator;

            switch (expr->op()) {
            case UnaryExpression::Op::NEGATIVE: m_accumulator = -value; break;
            case UnaryExpression::Op::POSITIVE: m_accumulator = value; break;
            case UnaryExpression::Op::ADDRESS_OF: throw LogicError("ADDRESS_OF not implemented yet");
            }
        }

        void visit(const GroupingExpression* expr, VisitorContext* ctx) override {
            expr->real()->accept(*this, nullptr);
        }

        void visit(const LiteralExpression* expr, VisitorContext* ctx) override {
            m_accumulator = expr->value();
        }

        // TODO: these
        void visit(const NumericVariableExpression* expr, VisitorContext* ctx) override { m_accumulator = 0.0; }
        void visit(const NamedVariableExpression* expr, VisitorContext* ctx) override { m_accumulator = 0.0; }
        void visit(const CallExpression* expr, VisitorContext* ctx) override { m_accumulator = 0.0; }

        // not used
        void visit(const CompoundStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate CompoundStatement")); }
        void visit(const BlockStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate BlockStatement")); }
        void visit(const SubStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate SubStatement")); }
        void visit(const IfStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate IfStatement")); }
        void visit(const WhileStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate WhileStatement")); }
        void visit(const ReturnStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate ReturnStatement")); }
        void visit(const AliasStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate AliasStatement")); }
        void visit(const LetStatement* stmt, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate LetStatement")); }
        void visit(const CommentExpression* expr, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate CommentExpression")); }
        void visit(const WordExpression* expr, VisitorContext* ctx) override { throw LogicError(std::format("tried to evaluate WordExpression")); }
    };
}

#endif //EVALUATOR_H
