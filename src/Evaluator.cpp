#include "evaluator/Evaluator.h"

#include <memory>
#include <concepts>
#include <functional>
#include <print>
#include <cmath>
#include <cstddef>
#include <format>
#include <vector>
#include <stdexcept>
#include <unordered_map>
#include <ranges>
#include <utility>

#include "parser/SubSignature.h"
#include "memory/Memory.h"
#include "gcode/GCode.h"

namespace ngc {
    class Evaluator::Impl final : public Visitor {
        std::vector<std::unordered_map<std::string_view, uint32_t>> m_scope;
        std::vector<std::unordered_map<SubSignature, const SubStatement *>> m_subScope;
        Memory &m_mem;
        const std::function<void(std::unique_ptr<const EvaluatorMessage>, Evaluator &)> &m_callback;
        Evaluator &m_owner;
        std::function<void()> m_interrupt;

        class Scope {
            Impl *m_evaluator = nullptr;
            bool m_opened = false;
            bool m_global = false;

        public:
            explicit Scope(Impl &evaluator, const bool global) : m_evaluator(&evaluator), m_global(global) { }
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

            [[nodiscard]] bool global() const { return m_global; }

            void allocate(const NamedVariableExpression *expr, const double value) {
                if(!m_evaluator) {
                    throw std::logic_error("Scope::allocate called on default constructed Scope");
                }

                if(!m_global && !m_opened) {
                    openScope();
                }

                if(m_evaluator->m_scope.size() == 1) {
                    if(m_evaluator->m_scope.front().contains(expr->name())) {
                        throw std::logic_error(std::format("redeclared global variable '{}'", expr->name()));
                    }

                    uint32_t addr = m_evaluator->m_mem.addData(MemoryCell(MemoryCell::Flags::READ | MemoryCell::Flags::WRITE, value));
                    m_evaluator->m_scope.front().emplace(expr->name(), addr);
                } else {
                    if(m_evaluator->m_scope.back().contains(expr->name())) {
                        throw std::logic_error(std::format("redeclared local variable '{}'", expr->name()));
                    }

                    uint32_t addr = m_evaluator->m_mem.push(value);
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
            double result = 0.0;
        };

        static Context *context(VisitorContext *ctx) { return static_cast<Context *>(ctx); }

    public:
        explicit Impl(Evaluator &owner, Memory &mem,
                           const std::function<void(std::unique_ptr<const EvaluatorMessage>, Evaluator &)> &callback,
                           std::function<void()> interrupt = {})
            : m_scope(1), m_subScope(1), m_mem(mem), m_callback(callback), m_owner(owner), m_interrupt(std::move(interrupt)) { }

        template<typename ...Args>
        double call(std::string name, Args... args) requires (std::convertible_to<Args, double> && ...) {
            std::vector<std::unique_ptr<ScalarExpression>> args2;
            (args2.emplace_back(LiteralExpression::fromDouble(args)), ...);
            return callImpl(std::move(name), std::move(args2));
        }

        void declareGlobal(const NamedVariableExpression *expr, uint32_t addr, const std::optional<double> value = std::nullopt) {
            if(m_scope.back().contains(expr->name())) {
                throw std::logic_error(std::format("already declared global variable '{}'", expr->name()));
            }

            if(addr == 0) {
                const auto _value = value ? *value : 0.0;
                addr = m_mem.addData(MemoryCell(MemoryCell::Flags::READ | MemoryCell::Flags::WRITE, _value));
            } else {
                if(value) {
                    write(addr, *value);
                }
            }

            m_scope.back().emplace(expr->name(), addr);
        }

        void executeFirstPass(const std::span<const Statement * const> program) {
            auto ctx = createScopeContext(true);

            for(const auto &stmt : program) {
                interrupt();
                if(const auto s = stmt->as<AliasStatement>(); s) {
                    const auto addr = static_cast<uint32_t>(eval(s->address(), &ctx));
                    declareGlobal(s->variable(), addr);
                    continue;
                }

                if(const auto s = stmt->as<LetStatement>(); s) {
                    const auto value = s->value() ? eval(s->value(), &ctx) : 0.0;
                    declareGlobal(s->variable(), 0, value);
                    continue;
                }

                if(const auto s = stmt->as<SubStatement>(); s) {
                    declareSub(s);
                }
            }
        }

        void executeSecondPass(const std::span<const Statement * const> program) {
            auto ctx = createScopeContext(true);

            for(const auto &stmt : program) {
                executeStatement(stmt, &ctx);
            }
        }

        void visit(const SubStatement* stmt, VisitorContext* ctx) override {
            if(context(ctx)->scope.global()) {
                return;
            }

            declareSub(stmt);
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
                executeStatement(s.get(), ctx);
                if(context(ctx)->action != Context::NONE) break;
            }
        }

        void visit(const AliasStatement *stmt, VisitorContext *ctx) override {
            if(context(ctx)->scope.global()) {
                return;
            }

            auto addr = static_cast<uint32_t>(eval(stmt->address(), ctx));
            m_scope.back().emplace(stmt->variable()->name(), addr);
        }

        void visit(const LetStatement* stmt, VisitorContext* ctx) override {
            if(context(ctx)->scope.global()) {
                return;
            }

            double value = 0.0;

            if(stmt->value()) {
                value = eval(stmt->value(), ctx);
            }

            context(ctx)->scope.allocate(stmt->variable(), value);
        }

        void visit(const BlockStatement* stmt, VisitorContext* ctx) override {
            std::vector<Word> words;

            for(const auto &expr : stmt->expressions()) {
                const Letter letter = convertLetter(expr->token().kind());
                const double real = eval(expr->real(), ctx);
                words.emplace_back(expr.get(), letter, real);
            }

            m_callback(std::make_unique<BlockMessage>(Block(stmt, std::move(words))), m_owner);
        }

        void visit(const ReturnStatement* stmt, VisitorContext* ctx) override {
            context(ctx)->result = eval(stmt->real(), ctx);
            context(ctx)->action = Context::RETURN;
        }

        void visit(const BreakStatement *, VisitorContext* ctx) override {
            context(ctx)->action = Context::BREAK;
        }

        void visit(const ContinueStatement *, VisitorContext* ctx) override {
            context(ctx)->action = Context::CONTINUE;
        }

        void visit(const IfStatement* stmt, VisitorContext* ctx) override {
            if(eval(stmt->condition(), ctx) != 0.0) {
                auto newCtx = createScopeContext();
                executeStatement(stmt->body(), &newCtx);
                propagateAction(*context(ctx), newCtx);
            } else {
                if(stmt->elseBody()) {
                    auto newCtx = createScopeContext();
                    executeStatement(stmt->elseBody(), &newCtx);
                    propagateAction(*context(ctx), newCtx);
                }
            }
        }

        void visit(const WhileStatement* stmt, VisitorContext* ctx) override {
            while(eval(stmt->condition(), ctx) != 0.0) {
                auto newCtx = createScopeContext();
                executeStatement(stmt->body(), &newCtx);

                if(newCtx.action == Context::RETURN) {
                    propagateAction(*context(ctx), newCtx);
                    return;
                }
                if(newCtx.action == Context::BREAK) return;
            }
        }

        void visit(const CallExpression* expr, VisitorContext* ctx) override {
            // functions implicitly return 0 if they dont return explicitly
            context(ctx)->result = 0.0;

            // TODO: more extensible way to evaluate built in functions
            if (expr->name() == "print" || expr->name() == "alert") {
                synchronize();
                std::string text;

                for(const auto &arg : expr->args()) {
                    if(const auto real = arg->as<RealExpression>()) {
                        text += std::format("{}", eval(real, ctx));
                    } else if(const auto str = arg->as<StringExpression>()) {
                        text += str->value();
                    }
                }

                if (expr->name() == "alert") {
                    m_callback(std::make_unique<AlertMessage>(text), m_owner);
                } else {
                    m_callback(std::make_unique<PrintMessage>(text), m_owner);
                }
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

            if(!stmt) {
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
                const auto value = eval(arg, ctx);
                newCtx.scope.allocate(params[i].get(), value);
            }

            for(const auto &s : stmt->body()->statements()) {
                executeStatement(s.get(), &newCtx);

                if(newCtx.action == Context::RETURN) {
                    context(ctx)->result = newCtx.result;
                    break;
                }
            }
        }

        void visit(const NumericVariableExpression* expr, VisitorContext* ctx) override {
            const auto addr = static_cast<uint32_t>(eval(expr->real(), ctx));
            context(ctx)->result = addr;
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
                throw std::logic_error(std::format("undeclared variable '{}'", expr->name()));
            }

            context(ctx)->result = addr;
        }

        void visit(const BinaryExpression* expr, VisitorContext* ctx) override {
            double left;
            double right;

            if(expr->op() == BinaryExpression::Op::ASSIGN) {
                if(!expr->left()->is<VariableExpression>()) {
                    throw std::logic_error(std::format("tried to assign to {}", expr->className()));
                }

                left = eval(expr->left(), ctx, false);
                right = eval(expr->right(), ctx);
                const auto addr = static_cast<uint32_t>(left);
                write(addr, right);
            } else {
                left = eval(expr->left(), ctx);
                right = eval(expr->right(), ctx);
            }

            double result;

            switch (expr->op()) {
                using enum BinaryExpression::Op;
                case ASSIGN: result = right; break;
                case AND: result = static_cast<bool>(left) && static_cast<bool>(right); break;
                case OR: result = static_cast<bool>(left) || static_cast<bool>(right); break;
                case XOR: result = (static_cast<bool>(left) && !static_cast<bool>(right)) || (static_cast<bool>(right) && !static_cast<bool>(left)); break;
                case EQ: result = left == right; break;
                case NE: result = left != right; break;
                case LT: result = left < right; break;
                case LE: result = left <= right; break;
                case GT: result = left > right; break;
                case GE: result = left >= right; break;
                case ADD: result = left + right; break;
                case SUB: result = left - right; break;
                case MUL: result = left * right; break;
                case DIV: result = left / right; break;
                case MOD: result = std::fmod(left, right); break;
                default: throw std::logic_error("missing ");
            }

            context(ctx)->result = result;
        }

        void visit(const UnaryExpression* expr, VisitorContext* ctx) override {
            double value;

            if(expr->op() == UnaryExpression::Op::ADDRESS_OF) {
                if(!expr->real()->is<VariableExpression>()) {
                    throw std::runtime_error(std::format("tried to take address of {}", expr->className()));
                }

                value = static_cast<uint32_t>(eval(expr->real(), ctx, false));
            } else {
                value = eval(expr->real(), ctx);
            }

            double result;

            switch (expr->op()) {
                case UnaryExpression::Op::NEGATIVE: result = -value; break;
                case UnaryExpression::Op::POSITIVE:
                case UnaryExpression::Op::ADDRESS_OF: result = value; break;
                default: throw std::runtime_error(std::format("invalid unary operator UnaryExpression::Op{}", std::to_underlying(expr->op())));
            }

            context(ctx)->result = result;
        }

        void visit(const GroupingExpression* expr, VisitorContext* ctx) override {
            context(ctx)->result = eval(expr->real(), ctx);
        }

        void visit(const LiteralExpression* expr, VisitorContext* ctx) override {
            context(ctx)->result = expr->value();
        }

        // not needed
        void visit(const StringExpression*, VisitorContext*) override { }
        void visit(const CommentExpression*, VisitorContext*) override { }
        void visit(const WordExpression*, VisitorContext*) override { }

    private:
        static void propagateAction(Context &parent, const Context &child) {
            parent.action = child.action;
            if(child.action == Context::RETURN) parent.result = child.result;
        }

        Context createScopeContext(const bool global = false) { return Context { .scope = Scope(*this, global) }; }

    public:
        double callImpl(std::string name, std::vector<std::unique_ptr<ScalarExpression>> args) {
            const auto startToken = Token::fromString(Token::Kind::IDENTIFIER, std::move(name));
            const auto endToken = Token::fromString(Token::Kind::RBRACKET, "]");
            const auto call = std::make_unique<CallExpression>(startToken, endToken, std::move(args));
            auto ctx = createScopeContext(true);
            return eval(call.get(), &ctx);
        }

        void declareSub(const SubStatement *stmt) {
            auto sig = SubSignature(stmt);

            if(m_subScope.back().contains(sig)) {
                throw std::logic_error(std::format("redeclared subroutine '{}'", sig.name()));
            }

            m_subScope.back().emplace(sig, stmt);
        }

        void executeStatement(const Statement *statement, VisitorContext *ctx) {
            interrupt();
            try {
                statement->accept(*this, ctx);
            } catch(const std::exception &error) {
                const auto location = statement->startToken().location();
                if(std::string_view(error.what()).starts_with(location)) throw;
                throw std::runtime_error(std::format("{}: {}", location, error.what()));
            }
        }

        void synchronize() {
            m_callback(std::make_unique<SynchronizationMessage>(), m_owner);
        }

        void pauseProgram() {
            m_callback(std::make_unique<ProgramPauseMessage>(), m_owner);
        }

        void toolChangeModalStateRestored() {
            m_callback(
                std::make_unique<ToolChangeModalStateRestoredMessage>(), m_owner);
        }

        void interrupt() const {
            if(m_interrupt) m_interrupt();
        }

        [[noreturn]] static void throwMemoryError(const Memory::Error error) {
            switch(error) {
                case Memory::Error::INVALID_DATA_ADDRESS:
                    throw std::logic_error("INVALID_DATA_ADDRESS");
                case Memory::Error::INVALID_STACK_ADDRESS:
                    throw std::logic_error("INVALID_STACK_ADDRESS");
                case Memory::Error::READ:
                    throw std::logic_error("READ");
                case Memory::Error::WRITE:
                    throw std::logic_error("WRITE");
            }
            throw std::logic_error("unknown memory error");
        }

        double eval(const RealExpression *expr, VisitorContext *ctx, const bool dereference = true) {
            if(dereference && expr->is<VariableExpression>()) synchronize();
            expr->accept(*this, ctx);

            if(dereference && expr->is<VariableExpression>()) {
                const auto addr = static_cast<uint32_t>(context(ctx)->result);
                return read(addr);
            }

            return context(ctx)->result;
        }

        bool isVolatile(const uint32_t addr) {
            const auto result = m_mem.isVolatile(addr);
            if(!result) throwMemoryError(result.error());
            return *result;
        }

        [[nodiscard]] double read(const uint32_t addr) {
            auto result = m_mem.read(addr);
            if(!result) throwMemoryError(result.error());
            return *result;
        }

        void write(const uint32_t addr, const double value) {
            if(auto result = m_mem.write(addr, value); !result)
                throwMemoryError(result.error());
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

    Evaluator::Evaluator(Memory &memory, const Callback &callback, std::function<void()> interrupt)
        : m_impl(std::make_unique<Impl>(*this, memory, callback, std::move(interrupt))) { }
    Evaluator::~Evaluator() = default;
    double Evaluator::callPrepared(std::string name, std::vector<std::unique_ptr<ScalarExpression>> args) {
        return m_impl->callImpl(std::move(name), std::move(args));
    }
    void Evaluator::declareGlobal(const NamedVariableExpression *expression, const std::uint32_t address,
                                  const std::optional<double> value) {
        m_impl->declareGlobal(expression, address, value);
    }
    void Evaluator::executeFirstPass(const std::span<const Statement * const> program) {
        m_impl->executeFirstPass(program);
    }
    void Evaluator::executeSecondPass(const std::span<const Statement * const> program) {
        m_impl->executeSecondPass(program);
    }
    void Evaluator::synchronize() {
        m_impl->synchronize();
    }

    void Evaluator::pauseProgram() {
        m_impl->pauseProgram();
    }

    void Evaluator::toolChangeModalStateRestored() {
        m_impl->toolChangeModalStateRestored();
    }
}
