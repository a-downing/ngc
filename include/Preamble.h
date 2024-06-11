#ifndef PREAMBLE_H
#define PREAMBLE_H

#include <cstdint>
#include <string>
#include <utility>

#include <Vars.h>
#include <MemoryCell.h>
#include <CharacterSource.h>
#include <Lexer.h>
#include <Parser.h>
#include <Statement.h>

namespace ngc
{
    class Preamble {
        CharacterSource m_source;
        std::unique_ptr<CompoundStatement> m_statements;

        explicit Preamble(CharacterSource source) : m_source(std::move(source)) {
            auto lexer = Lexer(m_source);
            auto parser = Parser(lexer);
            m_statements = parser.parse();
        }

    public:
        Preamble(const Preamble &) = delete;
        Preamble(Preamble &&) = delete;
        Preamble &operator=(const Preamble &) = delete;
        Preamble &operator=(Preamble &&) = delete;

        [[nodiscard]] const CharacterSource &source() const { return m_source; }
        [[nodiscard]] const CompoundStatement *statements() const { return m_statements.get(); }

        static Preamble make(const std::initializer_list<std::tuple<Var, std::string_view, MemoryCell::Flags>> &vars, const std::vector<uint32_t> &addrs) {
            std::string text = "%\n";

            for(auto i = 0; auto &[var, name, flags] : vars) {
                text += std::format("alias #{} = {}\n", name, addrs[i]);
                i++;
            }

            text += '%';

            return Preamble(CharacterSource(text, "preamble"));
        }
    };
}

#endif //PREAMBLE_H
