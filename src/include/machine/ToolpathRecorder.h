#pragma once

#include <functional>
#include <utility>
#include <vector>

#include "machine/MachineCommand.h"

namespace ngc {
    class ToolpathRecorder {
        std::vector<MachineCommand> m_commands;

    public:
        void clear() { m_commands.clear(); }

        void consume(MachineCommand command) {
            m_commands.emplace_back(std::move(command));
        }

        const std::vector<MachineCommand> &commands() const { return m_commands; }

        void foreachCommand(const std::function<void(const MachineCommand &)> &callback) const {
            for(const auto &command : m_commands) {
                callback(command);
            }
        }
    };
}
