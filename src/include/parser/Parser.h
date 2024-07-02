#pragma once

#include <print>
#include <source_location>
#include <stdexcept>
#include <vector>
#include <memory>
#include <tuple>
#include <stack>

#include "parser/Lexer.h"
#include "parser/Expression.h"
#include "parser/Statement.h"

namespace ngc
{
    class Parser final {
        Lexer &m_lexer;
        bool m_percentFirst = false;
        bool m_finished = false;
        std::stack<bool> m_skipNewlines;

        bool skipNewlines() const { return m_skipNewlines.top(); }

        class ScopedSkipNewlines {
            Parser &m_parser;

        public:
            ScopedSkipNewlines(Parser &p, const bool skipNewlines) : m_parser(p) { m_parser.m_skipNewlines.push(skipNewlines); }
            ~ScopedSkipNewlines() { m_parser.m_skipNewlines.pop(); }
        };

    public:
        explicit Parser(Lexer &lexer): m_lexer(lexer), m_skipNewlines({true}) { }

        class Error final : public std::logic_error {
            std::source_location m_location;
            std::optional<Token> m_token;
            std::optional<Lexer::Error> m_lexerError;
            std::unique_ptr<Expression> m_expression;

        public:
            Error(Error &&err) = default;
            Error(const std::string &message, const std::source_location location): std::logic_error(message), m_location(location) { }
            Error(const std::string &message, const std::source_location location, const Token &token): std::logic_error(message), m_location(location), m_token(token) { }
            Error(const std::string &message, const std::source_location location, const Lexer::Error &lexerError): std::logic_error(message), m_location(location), m_lexerError(lexerError) { }
            Error(const std::string &message, const std::source_location location, std::unique_ptr<Expression> expression): std::logic_error(message), m_location(location), m_expression(std::move(expression)) { }

            [[nodiscard]] const std::source_location &sourceLocation() const { return m_location; }
            [[nodiscard]] const std::optional<Token> &token() const { return m_token; }
            [[nodiscard]] const std::optional<Lexer::Error> &lexerError() const { return m_lexerError; }
            [[nodiscard]] Expression *expression() const { return m_expression.get(); }

            std::string text() const {
                if(m_lexerError) {
                    return std::format("{}: {}", m_lexerError->location(), m_lexerError->message());
                }

                if(m_token) {
                    return std::format("{}: {}", m_token->location(), what());
                }

                return what();
            }
        };

        std::expected<std::vector<std::unique_ptr<Statement>>, Error> parse() {
            std::vector<std::unique_ptr<Statement>> statements;

            try {
                if(match(Token::Kind::PERCENT)) {
                    m_percentFirst = true;
                }

                while(auto statement = parseStatement()) {
                    statements.emplace_back(std::move(statement));
                }
            } catch(Error &err) {
                return std::unexpected(std::move(err));
            }

            return { std::move(statements) };
        }

    private:
        std::unique_ptr<CompoundStatement> parseCompoundStatement() {
            auto startToken = expect(Token::Kind::LBRACE);
            std::vector<std::unique_ptr<Statement>> statements;

            while(auto statement = parseStatement()) {
                statements.emplace_back(std::move(statement));
            }

            Token endToken = expect(Token::Kind::RBRACE);
            return std::make_unique<CompoundStatement>(std::move(startToken), std::move(endToken), std::move(statements));
        }

        [[nodiscard]] std::unique_ptr<Statement> parseStatement() {
            if(m_finished || match(Token::Kind::NONE)) {
                return nullptr;
            }

            Token token;

            if(match(token, Token::Kind::PERCENT)) {
                if(m_percentFirst) {
                    m_finished = true;
                    return nullptr;
                }

                error("unexpected token", token);
            }

            if(check(Token::Kind::LBRACE)) {
                return parseCompoundStatement();
            }

            if(check(Token::Kind::RBRACE)) {
                return nullptr;
            }

            if(check(Token::Kind::SUB)) {
                return parseSubStatement();
            }

            if(check(Token::Kind::IF)) {
                return parseIfStatement();
            }

            if(check(Token::Kind::WHILE)) {
                return parseWhileStatement();
            }

            if(match(token, Token::Kind::BREAK)) {
                return std::make_unique<BreakStatement>(token);
            }

            if(match(token, Token::Kind::CONTINUE)) {
                return std::make_unique<ContinueStatement>(token);
            }

            if(match(token, Token::Kind::RETURN)) {
                return std::make_unique<ReturnStatement>(token, expect<RealExpression>(parseExpression()));
            }

            if(check(Token::Kind::ALIAS)) {
                return parseAliasStatement();
            }

            if(check(Token::Kind::LET)) {
                return parseLetStatement();
            }

            token = peekToken();

            if(token.kind() == Token::Kind::SLASH || token.isLetter()) {
                return parseBlockStatement();
            }

            return std::make_unique<ExpressionStatement>(parseExpression());
        }

        [[nodiscard]] std::unique_ptr<BlockStatement> parseBlockStatement() {
            Token token;
            std::optional<Token> blockDelete = std::nullopt;

            ScopedSkipNewlines skip(*this, false);

            // eat up any leading newlines
            while(match(Token::Kind::NEWLINE)) {

            }

            if(match(token, Token::Kind::SLASH)) {
                blockDelete.emplace(token);
            }

            std::vector<std::unique_ptr<WordExpression>> expressions;

            for(;;) {
                if(match(Token::Kind::NEWLINE, Token::Kind::NONE)) {
                    auto block = std::make_unique<BlockStatement>(std::move(blockDelete), std::move(expressions));
                    return block;
                }

                expressions.emplace_back(expect<WordExpression>(parsePrimaryExpression()));
            }
        }

        [[nodiscard]] std::unique_ptr<SubStatement> parseSubStatement() {
            auto startToken = expect(Token::Kind::SUB);
            auto identifier = expect(Token::Kind::IDENTIFIER);
            std::ignore = expect(Token::Kind::LBRACKET);

            auto params = std::vector<std::unique_ptr<NamedVariableExpression>>();

            if(!check(Token::Kind::RBRACKET)) {
                do {
                    params.emplace_back(expect<NamedVariableExpression>(parseExpression()));
                } while(match(Token::Kind::COMMA));
            }

            std::ignore = expect(Token::Kind::RBRACKET);
            return std::make_unique<SubStatement>(startToken, identifier, std::move(params), parseCompoundStatement());
        }

        [[nodiscard]] std::unique_ptr<IfStatement> parseIfStatement() {
            auto startToken = expect(Token::Kind::IF);
            auto condition = expect<RealExpression>(parseExpression());
            auto statements = parseCompoundStatement();

            std::unique_ptr<CompoundStatement> elseStatements;

            if(match(Token::Kind::ELSE)) {
                elseStatements = parseCompoundStatement();
            }

            return std::make_unique<IfStatement>(startToken, std::move(condition), std::move(statements), std::move(elseStatements));
        }

        [[nodiscard]] std::unique_ptr<WhileStatement> parseWhileStatement() {
            auto startToken = expect(Token::Kind::WHILE);
            auto condition = expect<RealExpression>(parseExpression());
            auto statements = parseCompoundStatement();
            return std::make_unique<WhileStatement>(startToken, std::move(condition), std::move(statements));
        }

        [[nodiscard]] std::unique_ptr<AliasStatement> parseAliasStatement() {
            auto startToken = expect(Token::Kind::ALIAS);
            auto namedVariable = expect<NamedVariableExpression>(parsePrimaryExpression());
            std::ignore = expect(Token::Kind::ASSIGN);
            return std::make_unique<AliasStatement>(startToken, std::move(namedVariable), expect<RealExpression>(parseExpression()));
        }

        [[nodiscard]] std::unique_ptr<LetStatement> parseLetStatement() {
            auto startToken = expect(Token::Kind::LET);
            auto namedVariable = expect<NamedVariableExpression>(parsePrimaryExpression());
            std::unique_ptr<RealExpression> realExpression;

            if(match(Token::Kind::ASSIGN)) {
                realExpression = expect<RealExpression>(parseExpression());
            }

            return std::make_unique<LetStatement>(startToken, std::move(namedVariable), std::move(realExpression));
        }

        [[nodiscard]] std::unique_ptr<Expression> parseExpression() {
            auto expr = parseAssignmentExpression();
            //std::println("expr: {} {}", expr->className(), expr->text());
            return expr;
        }

        [[nodiscard]] std::unique_ptr<Expression> parseAssignmentExpression() {
            auto expression = parseOrXorExpression();

            if(Token token; match(token, Token::Kind::ASSIGN)) {
                auto left = expect<VariableExpression>(std::move(expression));
                auto right = expect<RealExpression>(parseAssignmentExpression());
                return std::make_unique<BinaryExpression>(token, std::move(left), std::move(right));
            }

            return expression;
        }

        [[nodiscard]] std::unique_ptr<Expression> parseOrXorExpression() {
            auto expression = parseAndExpression();
            Token token;

            while(match(token, Token::Kind::OR, Token::Kind::XOR)) {
                auto left = expect<RealExpression>(std::move(expression));
                auto right = expect<RealExpression>(parseAndExpression());
                expression = std::make_unique<BinaryExpression>(token, std::move(left), std::move(right));
            }

            return expression;
        }

        [[nodiscard]] std::unique_ptr<Expression> parseAndExpression() {
            auto expression = parseComparisonExpression();
            Token token;

            while(match(token, Token::Kind::AND)) {
                auto left = expect<RealExpression>(std::move(expression));
                auto right = expect<RealExpression>(parseComparisonExpression());
                expression = std::make_unique<BinaryExpression>(token, std::move(left), std::move(right));
            }

            return expression;
        }

        [[nodiscard]] std::unique_ptr<Expression> parseComparisonExpression() {
            auto expression = parseAddSubExpression();
            Token token;

            while(match(token, Token::Kind::EQ, Token::Kind::NE, Token::Kind::LT, Token::Kind::LE, Token::Kind::GT, Token::Kind::GE)) {
                auto left = expect<RealExpression>(std::move(expression));
                auto right = expect<RealExpression>(parseAddSubExpression());
                expression = std::make_unique<BinaryExpression>(token, std::move(left), std::move(right));
            }

            return expression;
        }

        [[nodiscard]] std::unique_ptr<Expression> parseAddSubExpression() {
            auto expression = parseMulDivModExpression();
            Token token;

            while(match(token, Token::Kind::PLUS, Token::Kind::MINUS)) {
                auto left = expect<RealExpression>(std::move(expression));
                auto right = expect<RealExpression>(parseMulDivModExpression());
                expression = std::make_unique<BinaryExpression>(token, std::move(left), std::move(right));
            }

            return expression;
        }

        [[nodiscard]] std::unique_ptr<Expression> parseMulDivModExpression() {
            auto expression = parseUnaryExpression();
            Token token;

            while(match(token, Token::Kind::MUL, Token::Kind::SLASH, Token::Kind::MOD)) {
                auto left = expect<RealExpression>(std::move(expression));
                auto right = expect<RealExpression>(parseUnaryExpression());
                expression = std::make_unique<BinaryExpression>(token, std::move(left), std::move(right));
            }

            return expression;
        }

        [[nodiscard]] std::unique_ptr<Expression> parseUnaryExpression() {
            if(Token token; match(token, Token::Kind::PLUS, Token::Kind::MINUS)) {
                return std::make_unique<UnaryExpression>(token, expect<RealExpression>(parseUnaryExpression()));
            }

            return parsePrimaryExpression();
        }

        [[nodiscard]] std::unique_ptr<Expression> parsePrimaryExpression() {
            const auto token = nextToken();

            if(token.is(Token::Kind::COMMENT)) {
                return std::make_unique<CommentExpression>(token);
            }

            if(token.isLetter()) {
                return std::make_unique<WordExpression>(token, expect<RealExpression>(parseExpression()));
            }

            if(token.is(Token::Kind::NUMBER)) {
                return std::make_unique<LiteralExpression>(token);
            }

            if(token.is(Token::Kind::POUND)) {
                return std::make_unique<NumericVariableExpression>(token, expect<RealExpression>(parsePrimaryExpression()));
            }

            if(token.is(Token::Kind::NAMED_VARIABLE)) {
                return std::make_unique<NamedVariableExpression>(token);
            }

            if(token.is(Token::Kind::IDENTIFIER)) {
                if(check(Token::Kind::LBRACKET)) {
                    return parseCallExpression(token);
                }
            }

            if(token.is(Token::Kind::LBRACKET)) {
                auto expression = expect<RealExpression>(parseExpression());
                auto endToken = expect(Token::Kind::RBRACKET);
                return std::make_unique<GroupingExpression>(token, endToken, std::move(expression));
            }

            if(token.is(Token::Kind::AMPERSAND)) {
                return std::make_unique<UnaryExpression>(token, expect<VariableExpression>(parseExpression()));
            }

            if(token.is(Token::Kind::STRING)) {
                return std::make_unique<StringExpression>(token);
            }

            error("unexpected token", token);
        }

        [[nodiscard]] std::unique_ptr<CallExpression> parseCallExpression(const Token &token) {
            std::ignore = expect(Token::Kind::LBRACKET);
            auto args = std::vector<std::unique_ptr<ScalarExpression>>();

            if(!check(Token::Kind::RBRACKET)) {
                do {
                    args.emplace_back(expect<ScalarExpression>(parseExpression()));
                } while(match(Token::Kind::COMMA));
            }

            auto endToken = expect(Token::Kind::RBRACKET);
            return std::make_unique<CallExpression>(token, endToken, std::move(args));
        }

        template<typename T>
        [[nodiscard]] std::unique_ptr<T> expect(std::unique_ptr<Expression> expression) {
            if(!expression->is<T>()) {
                auto message = std::format("expected {}, but found {}: '{}'", T::staticClassName(), expression->className(), expression->text());
                error(message, std::move(expression));
            }

            return std::unique_ptr<T>(static_cast<T *>(expression.release()));
        }

        [[nodiscard]] Token expect(const Token::Kind kind) {
            Token token;

            if(!match(token, kind)) {
                error(std::format("expected {}, but found '{}'", name(kind), peekToken().text()), peekToken());
            }

            return token;
        }

        [[nodiscard]] Token expectInteger() {
            const auto token = nextToken();

            if(!token.integer()) {
                error(std::format("expected integer, but found: '{}'", token.text()), token);
            }

            if(match(Token::Kind::A, Token::Kind::B)) {

            }

            return token;
        }

        template<typename ...Args>
        [[nodiscard]] bool match(Args... args) requires (std::same_as<Args, Token::Kind> && ...) {
            if((check(args) || ...)) {
                std::ignore = nextToken();
                return true;
            }

            return false;
        }

        template<typename ...Args>
        [[nodiscard]] bool match(Token &token, Args... args) requires (std::same_as<Args, Token::Kind> && ...) {
            if((check(args) || ...)) {
                token = nextToken();
                return true;
            }

            return false;
        }

        template<typename ...Args>
        [[nodiscard]] bool check(Args... args) requires (std::same_as<Args, Token::Kind> && ...) {
            if((peekToken().is(args) || ...)) {
                return true;
            }

            return false;
        }

        [[nodiscard]] Token peekToken() {
            m_lexer.pushState();

            for(;;) {
                const auto token = m_lexer.nextToken();

                if(!token) {
                    m_lexer.popState();
                    error("lexer error", token.error());
                }

                if(token->is(Token::Kind::NEWLINE) && skipNewlines()) {
                    continue;
                }

                m_lexer.popState();
                return *token;
            }
        }

        [[nodiscard]] Token nextToken() const {
            for(;;) {
                const auto token = m_lexer.nextToken();
                //std::println("TOKEN: {} '{}'", token->name(), token->text());

                if(!token) {
                    error("lexer error", token.error());
                }

                if(token->is(Token::Kind::NEWLINE) && skipNewlines()) {
                    continue;
                }

                return *token;
            }
        }

        [[noreturn]] static void error(const std::string &message, const std::source_location location = std::source_location::current()) {
            throw Error(message, location);
        }

        [[noreturn]] static void error(const std::string &message, const Token &token, const std::source_location location = std::source_location::current()) {
            throw Error(message, location, token);
        }

        [[noreturn]] static void error(const std::string &message, const Lexer::Error &lexerError, const std::source_location location = std::source_location::current()) {
            throw Error(message, location, lexerError);
        }

        [[noreturn]] static void error(const std::string &message, std::unique_ptr<Expression> expression, const std::source_location location = std::source_location::current()) {
            throw Error(message, location, std::move(expression));
        }
    };
}
