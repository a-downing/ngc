#pragma once

#include <memory>
#include <print>
#include <utility>

#include "parser/LexerSource.h"
#include "parser/Lexer.h"
#include "parser/Parser.h"
#include "parser/Statement.h"

namespace ngc {
    class Program {
        LexerSource m_source;
        std::vector<std::unique_ptr<Statement>> m_statements;
        std::vector<const Statement *> m_ptrStatements;
        bool m_compiled = false;

    public:
        Program(const Program &) = delete;
        Program(Program &&) = default;
        Program &operator=(const Program &) = delete;
        Program &operator=(Program &&) = default;

        explicit Program(LexerSource source) : m_source(std::move(source)) { }

        bool compiled() const { return m_compiled; }

        std::expected<std::span<const Statement * const>, Parser::Error> compile() {
            m_source.reset();

            auto lexer = Lexer(m_source);
            auto parser = Parser(lexer);
            auto result = parser.parse();

            if(!result) {
                return std::unexpected(std::move(result.error()));
            }

            m_statements = std::move(*result);

            for(const auto &statement : m_statements) {
                m_ptrStatements.emplace_back(statement.get());
            }

            m_compiled = true;
            return m_ptrStatements;
        }

        [[nodiscard]] std::span<const Statement * const> statements() const { return m_ptrStatements; }
        [[nodiscard]] const LexerSource &source() const { return m_source; }
    };
}
