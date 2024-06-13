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

        for(size_t i = 0; const auto &[var, name, flags] : VARS) {
            auto startToken = Token(Token::Kind::ALIAS, std::make_unique<StringTokenSource>("alias", ""));
            auto variableToken = Token(Token::Kind::NAMED_VARIABLE, std::make_unique<StringTokenSource>(std::format("#{}", name), ""));
            auto variable = std::make_unique<NamedVariableExpression>(variableToken);
            auto literalToken = Token(Token::Kind::NUMBER, std::make_unique<StringTokenSource>(std::to_string(addrs[i]), ""));
            auto alias = std::make_unique<AliasStatement>(startToken, std::move(variable), std::make_unique<LiteralExpression>(literalToken));
            statements.emplace_back(std::move(alias));
            i++;
        }

        return statements;
    }
}

#endif //PREAMBLE_H
