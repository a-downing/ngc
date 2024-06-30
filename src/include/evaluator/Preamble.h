#pragma once

#include <memory>
#include <vector>

#include "memory/Memory.h"
#include "memory/Vars.h"
#include "parser/Statement.h"

namespace ngc {
    class Preamble {
        std::vector<std::unique_ptr<Statement>> m_statements;
        std::vector<const Statement *> m_ptrStatements;

    public:
        Preamble(const Memory &mem) {
            const auto &addrs = mem.addrs();

            for(size_t i = 0; const auto &[var, name, addr, flags, value] : gVars) {
                auto startToken = Token::fromString(Token::Kind::ALIAS, "alias");
                auto variable = NamedVariableExpression::fromName(std::string(name));
                auto alias = std::make_unique<AliasStatement>(startToken, std::move(variable), LiteralExpression::fromDouble(addrs[i]));
                m_ptrStatements.emplace_back(alias.get());
                m_statements.emplace_back(std::move(alias));
                i++;
            }
        }

        std::span<const Statement * const> statements() const { return m_ptrStatements; }
    };
}
