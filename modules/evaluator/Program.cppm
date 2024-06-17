module;

#include <memory>
#include <utility>

export module evaluator:Program;
import parser;

export namespace ngc
{
    class Program {
        LexerSource m_source;
        std::vector<std::unique_ptr<Statement>> m_statements;

    public:
        Program(const Program &) = delete;
        Program(Program &&) = default;
        Program &operator=(const Program &) = delete;
        Program &operator=(Program &&) = default;

        explicit Program(LexerSource source) : m_source(std::move(source)) { }

        void compile() {
            auto lexer = Lexer(m_source);
            auto parser = Parser(lexer);
            m_statements = parser.parse();
        }

        [[nodiscard]] const std::vector<std::unique_ptr<Statement>> &statements() const { return m_statements; }
        [[nodiscard]] const LexerSource &source() const { return m_source; }
    };
}
