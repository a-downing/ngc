#include <print>
#include <fstream>
#include <filesystem>

 // for TestVisitor
#include <ranges>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include <Token.h>
#include <Lexer.h>
#include <Parser.h>
#include <Visitor.h>
#include <VisitorContext.h>

#include <Memory.h>

class SubSignature {
    std::string_view m_name;
    size_t m_numParams;

public:
    SubSignature(const std::string_view name, const size_t numParams) : m_name(name), m_numParams(numParams) { }
    explicit SubSignature(const ngc::SubStatement *stmt) : m_name(stmt->name()), m_numParams(stmt->params().size()) { }
    explicit SubSignature(const ngc::CallExpression *expr) : m_name(expr->name()), m_numParams(expr->args().size()) { }
    bool operator==(const SubSignature& s) const { return s.m_name == m_name && s.m_numParams == m_numParams; }
    [[nodiscard]] std::string_view name() const { return m_name; }
    [[nodiscard]] size_t numParams() const { return m_numParams; }
};

template<>
struct std::hash<SubSignature> {
    size_t operator()(const SubSignature &s) const noexcept {
        const auto h1 = std::hash<std::string_view>{}(s.name());
        const auto h2 = std::hash<size_t>{}(s.numParams());
        return h1 ^ (h2 << 1);
    }
};

class TestVisitor final : public ngc::Visitor {
    std::vector<std::unordered_map<std::string_view, const ngc::NamedVariableExpression *>> m_scope;
    std::vector<std::unordered_set<SubSignature>> m_subScope;

public:
    ~TestVisitor() override = default;

    TestVisitor() {
        m_scope.emplace_back();
        m_subScope.emplace_back();
    }

    void addGlobalSub(const SubSignature &s) {
        m_subScope.front().emplace(s);
    }

    bool isDeclared(const ngc::NamedVariableExpression *expr) const {
        return std::ranges::any_of(m_scope, [&expr](const auto &scope) {
            return scope.contains(expr->name());
        });
    }

    bool isDeclared(const ngc::CallExpression *expr) const {
        return std::ranges::any_of(std::views::reverse(m_subScope), [&expr](const auto &scope) {
            return scope.contains(SubSignature(expr));
        });
    }

    bool declare(const ngc::NamedVariableExpression *expr) {
        if(!m_scope.back().contains(expr->name())) {
            m_scope.back().emplace(expr->name(), expr);
            return true;
        }

        return false;
    }

    // statements
    void visit(const ngc::CompoundStatement *stmt, ngc::VisitorContext *ctx) override {
        for(const auto &s : stmt->statements()) {
            s->accept(*this, nullptr);
        }
    }

    void visit(const ngc::BlockStatement *stmt, ngc::VisitorContext *ctx) override {
        for(const auto &expr : stmt->expressions()) {
            expr->accept(*this, ctx);
        }
    }

    void visit(const ngc::SubStatement *stmt, ngc::VisitorContext *ctx) override {
        std::println("visitor: SubStatement '{}'", stmt->name());

        if(m_subScope.back().contains(SubSignature(stmt))) {
            std::println("{}: ERROR: redefinition of sub '{}'", stmt->startToken().location(), stmt->name());
        } else {
            m_subScope.back().insert(SubSignature(stmt));
        }

        std::unordered_map<std::string_view, const ngc::NamedVariableExpression *> paramNames;
        m_scope.emplace_back();

        for(const auto &param : stmt->params()) {
            if(paramNames.contains(param->name())) {
                std::println("{}: ERROR: parameter '{}' already declared", param->token().location(), param->name());
            }

            paramNames.emplace(param->name(), param.get());

            if(declare(param.get())) {
                std::println("{}: INFO: declared parameter variable '{}'", param->token().location(), param->name());
            }
        }

        if(stmt->body()) {
            for(const auto &s : stmt->body()->statements()) {
                s->accept(*this, ctx);
            }
        }

        m_scope.pop_back();
    }

    void visit(const ngc::AliasStatement *stmt, ngc::VisitorContext *ctx) override {
        std::println("visitor: AliasStatement '{}' {}", stmt->variable()->name(), stmt->address()->toString());
        m_scope.back().emplace(stmt->variable()->name(), stmt->variable());
        std::println("{}: INFO: declared alias variable '{}' -> {}", stmt->startToken().location(), stmt->variable()->name(), stmt->address()->toString());
    }

    void visit(const ngc::IfStatement *stmt, ngc::VisitorContext *ctx) override { }
    void visit(const ngc::WhileStatement *stmt, ngc::VisitorContext *ctx) override { }
    void visit(const ngc::ReturnStatement *stmt, ngc::VisitorContext *ctx) override { }

    // expressions
    void visit(const ngc::BinaryExpression *expr, ngc::VisitorContext *ctx) override {
        std::println("visitor: {}: {}: {}", expr->className(), expr->opName(), expr->toString());

        if(const auto named = expr->left()->as<ngc::NamedVariableExpression>(); named) {
            if(declare(named)) {
                std::println("{}: INFO: declared variable '{}'", named->token().location(), named->name());
            }
        }

        expr->left()->accept(*this, ctx);
        expr->right()->accept(*this, ctx);
    }

    void visit(const ngc::NamedVariableExpression *expr, ngc::VisitorContext *ctx) override {
        std::println("visitor: {}: var: {}", expr->className(), expr->name());

        if(!isDeclared(expr)) {
            std::println("{}: ERROR: variable '{}' accessed before assignment", expr->token().location(), expr->name());
        }
    }

    void visit(const ngc::CallExpression *expr, ngc::VisitorContext *ctx) override {
        if(!isDeclared(expr)) {
            std::println("{}: ERROR: call to '{}' before definition", expr->token().location(), expr->toString());
        }
    }

    void visit(const ngc::NumericVariableExpression *expr, ngc::VisitorContext *ctx) override { }
    void visit(const ngc::CommentExpression *expr, ngc::VisitorContext *ctx) override { }
    void visit(const ngc::WordExpression *expr, ngc::VisitorContext *ctx) override { }
    void visit(const ngc::LiteralExpression *expr, ngc::VisitorContext *ctx) override { }
    void visit(const ngc::UnaryExpression *expr, ngc::VisitorContext *ctx) override { }
    void visit(const ngc::GroupingExpression *expr, ngc::VisitorContext *ctx) override { }
};

std::string readFile(const std::filesystem::path& filePath) {
    std::ifstream file(filePath);

    if (!file) {
        throw std::ios_base::failure("Failed to open file");
    }

    file.seekg(0, std::ios::end);
    const auto fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::string fileContent(fileSize, '\0');
    file.read(&fileContent[0], fileSize);

    return fileContent;
}

int main(const int argc, const char **argv) {
    if(argc < 2) {
        std::println(stderr, "usage: {} <filename>", argv[0]);
        return 0;
    }

    const std::string filename = argv[1];
    const auto text = readFile(filename);
    auto source = ngc::CharacterSource(text, filename);
    auto lexer = ngc::Lexer(source);
    auto parser = ngc::Parser(lexer);

    std::unique_ptr<ngc::CompoundStatement> statements;

    try {
        statements = parser.parse();
    } catch (const ngc::Parser::Error &err) {
        if(err.lexerError()) {
            std::println(stderr, "{}: {}: {}", err.lexerError()->location(), err.what(), err.lexerError()->message());
        } else if(err.token()) {
            std::println(stderr, "{}: {}: {} '{}'", err.token()->location(), err.what(), err.token()->name(), err.token()->text());
        } else if(err.expression()) {
            std::println(stderr, "{}: {}", err.expression()->token().location(), err.what());
        } else {
            std::println(stderr, "{}", err.what());
        }

        std::println(stderr, "{}:{}:{}", err.sourceLocation().file_name(), err.sourceLocation().line(), err.sourceLocation().column());
        throw;
    }

    if(!statements) {
        std::println("empty program");
        return 0;
    }

    std::println("program has {} statements", statements->statements().size());

    // testing out how all this will work
    // TODO: will global variables declared in ngc files be in data or stack
    ngc::Memory mem;
    auto addrs = mem.init(ngc::GLOBALS);
    std::string preambleText = "%\n";

    for(auto i = 0; auto &[var, name, flags] : ngc::GLOBALS) {
        auto alias = std::format("alias #{} = {}", name, addrs[i]);
        std::println("alias: {}", alias);
        preambleText += alias + '\n';
        i++;
    }

    preambleText += "%";

    auto preambleSource = ngc::CharacterSource(preambleText, "preamble");
    auto preambleLexer = ngc::Lexer(preambleSource);
    auto preambleParser = ngc::Parser(preambleLexer);
    auto preamble = preambleParser.parse();

    TestVisitor visitor;
    visitor.addGlobalSub(SubSignature("sin", 1));
    preamble->accept(visitor, nullptr);
    statements->accept(visitor, nullptr);
}
