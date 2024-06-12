#ifndef PROGRAM_H
#define PROGRAM_H

#include <CharacterSource.h>
#include <Lexer.h>
#include <Parser.h>

#include <utility>

namespace ngc
{
    class Program {
        CharacterSource m_source;
        std::unique_ptr<CompoundStatement> m_statements;

    public:
        Program(const Program &) = delete;
        Program(Program &&) = default;
        Program &operator=(const Program &) = delete;
        Program &operator=(Program &&) = default;

        explicit Program(CharacterSource source) : m_source(std::move(source)) { }

        void compile() {
            auto lexer = Lexer(m_source);
            auto parser = Parser(lexer);
            m_statements = parser.parse();
        }

        [[nodiscard]] const CompoundStatement *statements() const { return m_statements.get(); }
        [[nodiscard]] const CharacterSource &source() const { return m_source; }
    };
}

#endif //PROGRAM_H
