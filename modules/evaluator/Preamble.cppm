module;

#include <memory>
#include <vector>

export module evaluator:Preamble;
import parser;
import memory;

export namespace ngc {
    inline std::vector<std::unique_ptr<Statement>> buildPreamble(const std::vector<uint32_t> &addrs) {
        auto statements = std::vector<std::unique_ptr<Statement>>();

        for(size_t i = 0; const auto &[var, name, addr, flags, value] : VARS) {
            auto startToken = Token::fromString(Token::Kind::ALIAS, "alias");
            auto variable = NamedVariableExpression::fromName(std::string(name));
            auto alias = std::make_unique<AliasStatement>(startToken, std::move(variable), LiteralExpression::fromDouble(addrs[i]));
            statements.emplace_back(std::move(alias));
            i++;
        }

        return statements;
    }
}
