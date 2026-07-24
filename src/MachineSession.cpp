#include "machine/MachineSession.h"

#include <stdexcept>

namespace ngc {
    SessionCommandQueue::SessionCommandQueue(const std::size_t capacity) : m_capacity(capacity) {
        if (capacity == 0) {
            throw std::invalid_argument("session command queue capacity must be positive");
        }
    }

    bool SessionCommandQueue::tryPush(SessionCommand command) {
        if (m_commands.size() >= m_capacity) {
            return false;
        }

        m_commands.push_back(std::move(command));

        return true;
    }

    std::optional<SessionCommand> SessionCommandQueue::tryPop() {
        if (m_commands.empty()) {
            return std::nullopt;
        }

        auto command = std::move(m_commands.front());
        m_commands.pop_front();

        return command;
    }

    void SessionCommandQueue::clear() noexcept {
        m_commands.clear();
    }

    bool SessionCommandQueue::empty() const noexcept {
        return m_commands.empty();
    }

    std::size_t SessionCommandQueue::size() const noexcept {
        return m_commands.size();
    }

    std::size_t SessionCommandQueue::capacity() const noexcept {
        return m_capacity;
    }
}
