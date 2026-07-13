#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "gcode/GCode.h"
#include "memory/Memory.h"
#include "parser/Expression.h"
#include "parser/Statement.h"

namespace ngc {
    class Evaluator;
    class BlockMessage;
    class PrintMessage;
    class SynchronizationMessage;

    class EvaluatorMessageVisitor {
    public:
        virtual ~EvaluatorMessageVisitor() = default;
        virtual void visit(const BlockMessage &) = 0;
        virtual void visit(const PrintMessage &) = 0;
        virtual void visit(const SynchronizationMessage &) = 0;
    };

    class EvaluatorMessage {
    public:
        virtual ~EvaluatorMessage() = default;
        virtual void accept(EvaluatorMessageVisitor &visitor) const = 0;
        virtual bool isImpl(const BlockMessage *) const { return false; }
        virtual bool isImpl(const PrintMessage *) const { return false; }
        virtual bool isImpl(const SynchronizationMessage *) const { return false; }

        template<typename T> const T *as() const {
            return isImpl(static_cast<const T *>(nullptr)) ? static_cast<const T *>(this) : nullptr;
        }
    };

    class BlockMessage final : public EvaluatorMessage {
        Block m_block;
    public:
        explicit BlockMessage(Block block) : m_block(std::move(block)) { }
        const Block &block() const { return m_block; }
        bool isImpl(const BlockMessage *) const override { return true; }
        void accept(EvaluatorMessageVisitor &visitor) const override { visitor.visit(*this); }
    };

    class PrintMessage final : public EvaluatorMessage {
        std::string m_text;
    public:
        explicit PrintMessage(std::string text) : m_text(std::move(text)) { }
        const std::string &text() const { return m_text; }
        bool isImpl(const PrintMessage *) const override { return true; }
        void accept(EvaluatorMessageVisitor &visitor) const override { visitor.visit(*this); }
    };

    class SynchronizationMessage final : public EvaluatorMessage {
    public:
        bool isImpl(const SynchronizationMessage *) const override { return true; }
        void accept(EvaluatorMessageVisitor &visitor) const override { visitor.visit(*this); }
    };

    class Evaluator {
    public:
        using Callback = std::function<void(std::unique_ptr<const EvaluatorMessage>, Evaluator &)>;

        explicit Evaluator(Memory &memory, const Callback &callback, std::function<void()> interrupt = {});
        ~Evaluator();
        Evaluator(const Evaluator &) = delete;
        Evaluator &operator=(const Evaluator &) = delete;

        template<typename ...Args>
        double call(std::string name, Args... args) requires (std::convertible_to<Args, double> && ...) {
            std::vector<std::unique_ptr<ScalarExpression>> values;
            (values.emplace_back(LiteralExpression::fromDouble(args)), ...);
            return callPrepared(std::move(name), std::move(values));
        }

        void declareGlobal(const NamedVariableExpression *expression, std::uint32_t address,
                           std::optional<double> value = std::nullopt);
        void executeFirstPass(std::span<const Statement * const> program);
        void executeSecondPass(std::span<const Statement * const> program);
        void synchronize();

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
        double callPrepared(std::string name, std::vector<std::unique_ptr<ScalarExpression>> args);
    };
}
