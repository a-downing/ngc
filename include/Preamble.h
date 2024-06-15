#ifndef PREAMBLE_H
#define PREAMBLE_H

#include <memory>
#include <vector>

#include <Token.h>
#include <Expression.h>
#include <Statement.h>
#include <Vars.h>

namespace ngc {
    inline std::vector<std::unique_ptr<Statement>> buildPreamble(const std::vector<uint32_t> &addrs) {
        auto statements = std::vector<std::unique_ptr<Statement>>();

        for(size_t i = 0; const auto &[var, name, addr, flags] : VARS) {
            auto startToken = Token::fromString(Token::Kind::ALIAS, "alias");
            auto variable = NamedVariableExpression::fromName(std::string(name));
            auto alias = std::make_unique<AliasStatement>(startToken, std::move(variable), LiteralExpression::fromDouble(addrs[i]));
            statements.emplace_back(std::move(alias));
            i++;
        }

        return statements;
    }
}

#endif //PREAMBLE_H
