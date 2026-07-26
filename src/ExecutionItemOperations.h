#pragma once

#include <cstdint>

#include "machine/MotionBackend.h"

namespace ngc::execution_item {
    [[nodiscard]] bool valid(const ExecutionItem &item) noexcept;
    [[nodiscard]] std::uint64_t normalMotionNanoseconds(const ExecutionItem &item) noexcept;
    [[nodiscard]] EpochId epoch(const ExecutionItem &item) noexcept;
    [[nodiscard]] ChunkId id(const ExecutionItem &item) noexcept;
    [[nodiscard]] BranchSequence predecessor(const ExecutionItem &item) noexcept;
}
