#ifndef FIRSTPASS_H
#define FIRSTPASS_H

#include <concepts>
#include <functional>
#include <print>
#include <cmath>
#include <cstddef>
#include <format>
#include <vector>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <ranges>

#include <Visitor.h>
#include <VisitorContext.h>
#include <Expression.h>
#include <Statement.h>
#include <MemoryCell.h>
#include <Memory.h>
#include <SubSignature.h>
#include <GCode.h>

namespace ngc
{
    class FirstPass  : public Visitor {
        std::vector<std::unordered_map<std::string_view, uint32_t>> m_scope;
        std::vector<std::unordered_map<SubSignature, const SubStatement *>> m_subScope;

    public:
        explicit FirstPass() : m_scope(1), m_subScope(1) { }

        void executeProgram(const std::vector<std::unique_ptr<Statement>> &program) {
            auto ctx = createScopeContext(true);

            for(const auto &stmt : program) {
                if(stmt->is<NamedVariableExpression>())
            }

            for(const auto &stmt : program) {
                stmt->accept(*this, &ctx);
            }

            while(!m_blocks.empty()) {
                m_callback(m_blocks, *this);
            }
        }


    };
}

#endif //FIRSTPASS_H
