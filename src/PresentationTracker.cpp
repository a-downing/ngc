#include "machine/PresentationTracker.h"

#include <algorithm>
#include <concepts>
#include <ranges>
#include <type_traits>
#include <utility>

namespace ngc {
    void PresentationTracker::reset(const TrajectoryCommandPresentation &initial) {
        m_snapshot = {};
        clearTracking();
        setActivePresentation(initial);
    }

    void PresentationTracker::clearTracking() {
        m_chunks.clear();
        m_markerPresentations.clear();
        m_executionSpans.clear();
        m_chunksWithMarkers.clear();
        m_indexedDiagnosticChunks.clear();
        m_blockChunks.clear();
        m_deferredCompletedBlocks.clear();
        m_retiredChunks.clear();
        m_interpretedBlocks.clear();
    }

    const MachinePresentationSnapshot &PresentationTracker::snapshot() const {
        return m_snapshot;
    }

    void PresentationTracker::setActivePresentation(const TrajectoryCommandPresentation &presentation) {
        m_snapshot.activePresentation = presentation;
        if (!presentation.workCoordinateSystem) {
            return;
        }

        const auto system = std::ranges::find_if(m_snapshot.usedWorkCoordinateSystems,
            [&](const auto &value) {
                return value.name == presentation.workCoordinateSystem->name;
            });
        if (system == m_snapshot.usedWorkCoordinateSystems.end()) {
            m_snapshot.usedWorkCoordinateSystems.push_back(*presentation.workCoordinateSystem);
        }
    }

    void PresentationTracker::setActiveWorkCoordinateSystem(const WorkCoordinateSystem &coordinateSystem) {
        m_snapshot.activePresentation.workCoordinateSystem = coordinateSystem;
        const auto existing = std::ranges::find_if(m_snapshot.usedWorkCoordinateSystems,
            [&](const auto &value) { return value.name == coordinateSystem.name; });
        if (existing == m_snapshot.usedWorkCoordinateSystems.end()) {
            m_snapshot.usedWorkCoordinateSystems.push_back(coordinateSystem);
        } else {
            *existing = coordinateSystem;
        }
    }

    void PresentationTracker::clearActiveTool() {
        m_snapshot.activePresentation.tool = {};
    }

    void PresentationTracker::completeBlock(const BlockExecution &block) {
        m_snapshot.completedBlocks.push_back(block);
        auto &flags = m_snapshot.completedLineFlags[block.source];
        if (block.line >= static_cast<int>(flags.size())) {
            flags.resize(static_cast<std::size_t>(block.line) + 1);
        }
        if (block.line >= 0) {
            flags[static_cast<std::size_t>(block.line)] = 1;
        }
    }

    void PresentationTracker::observeLifecycle(const InterpreterBlockLifecycle &lifecycle) {
        if (lifecycle.phase == BlockLifecyclePhase::Entered) {
            m_interpretedBlocks.push_back(lifecycle.block);
            return;
        }

        const auto match = std::ranges::find_if(m_interpretedBlocks,
            [&](const auto &block) { return block.id == lifecycle.block.id; });
        if (match != m_interpretedBlocks.end()) {
            m_interpretedBlocks.erase(match);
        }

        const auto owner = m_blockChunks.find(lifecycle.block.id);
        if (owner == m_blockChunks.end()) {
            const auto existing = std::ranges::find(
                m_deferredCompletedBlocks, lifecycle.block.id, &BlockExecution::id);
            if (existing == m_deferredCompletedBlocks.end()) {
                m_deferredCompletedBlocks.push_back(lifecycle.block);
            } else {
                *existing = lifecycle.block;
            }
        } else if (m_retiredChunks.contains(owner->second)) {
            completeBlock(lifecycle.block);
        } else {
            m_chunks[owner->second].completedBlocks.push_back(lifecycle.block);
        }
    }

    void PresentationTracker::observeCommand(const MachineCommand &command, const ExecutionItem &item,
                                             const TrajectoryCommandPresentation &captured,
                                             const ExecutionMarkerId activationMarker) {
        ChunkPresentation presentation;
        presentation.active = captured;
        presentation.command = command;
        const auto key = std::visit(
            [](const auto &value) { return ChunkKey {value.epoch, value.id}; }, item);

        if (const auto *chunk = std::get_if<PlanChunk>(&item);
            chunk && m_indexedDiagnosticChunks.insert(key).second) {
            const auto retainSpan = [&](const AxisPolynomialSpan &span,
                                        const std::uint32_t ordinal, const bool stopTail) {
                const auto start = executionSpanStart(span);
                const auto end = executionSpanEnd(span);
                m_executionSpans.try_emplace(SpanKey {chunk->epoch, span.id},
                    PresentationExecutionSpanDiagnostic {
                        .chunk = chunk->id,
                        .ordinal = ordinal,
                        .stopTail = stopTail,
                        .duration = span.duration,
                        .distance = (end.position - start.position).length(),
                        .startVelocity = start.velocity.length(),
                        .endVelocity = end.velocity.length(),
                        .startAcceleration = start.acceleration.length(),
                        .endAcceleration = end.acceleration.length(),
                    });
            };
            for (std::uint32_t index = 0; index < chunk->normalMotion.size; ++index) {
                retainSpan(chunk->normalMotion[index], index, false);
            }
            for (std::uint32_t index = 0; index < chunk->stopTail.size; ++index) {
                retainSpan(chunk->stopTail[index], index, true);
            }
        }

        for (const auto &block : captured.activeBlocks) {
            m_blockChunks.insert_or_assign(block.id, key);
        }
        for (auto completed = m_deferredCompletedBlocks.begin();
             completed != m_deferredCompletedBlocks.end();) {
            const auto owner = m_blockChunks.find(completed->id);
            if (owner != m_blockChunks.end() && owner->second == key) {
                presentation.completedBlocks.push_back(*completed);
                completed = m_deferredCompletedBlocks.erase(completed);
            } else {
                ++completed;
            }
        }

        if (activationMarker != 0) {
            m_markerPresentations.insert_or_assign(
                MarkerKey {key.first, activationMarker}, presentation);
            m_chunksWithMarkers.insert(key);
        }

        const auto existing = m_chunks.find(key);
        if (existing != m_chunks.end()) {
            presentation.completedBlocks.insert(presentation.completedBlocks.end(),
                existing->second.completedBlocks.begin(), existing->second.completedBlocks.end());
        }
        m_chunks.insert_or_assign(key, std::move(presentation));
    }

    void PresentationTracker::applyPresentation(const ChunkPresentation &presentation) {
        setActivePresentation(presentation.active);
        if (!presentation.command) {
            return;
        }

        std::visit([&](const auto &command) {
            using T = std::decay_t<decltype(command)>;
            if constexpr (std::same_as<T, SpindleStart>) {
                m_snapshot.spindleRunning = true;
                m_snapshot.spindleSpeed = command.speed();
                m_snapshot.spindleDirection = command.direction();
            } else if constexpr (std::same_as<T, SpindleStop>) {
                m_snapshot.spindleRunning = false;
                m_snapshot.spindleSpeed = 0.0;
            }
        }, *presentation.command);
    }

    void PresentationTracker::observeChunkAccepted(const ChunkAccepted &accepted) {
        const ChunkKey key {accepted.epoch, accepted.chunk};
        if (m_chunksWithMarkers.contains(key)) {
            return;
        }

        const auto found = m_chunks.find(key);
        if (found != m_chunks.end()) {
            applyPresentation(found->second);
        }
    }

    void PresentationTracker::observeMarkerReached(const ExecutionMarkerReached &marker) {
        const auto found = m_markerPresentations.find({marker.epoch, marker.marker});
        if (found != m_markerPresentations.end()) {
            applyPresentation(found->second);
        }
    }

    void PresentationTracker::observeChunkRetired(const ChunkRetired &retired) {
        const ChunkKey key {retired.epoch, retired.chunk};
        const auto found = m_chunks.find(key);
        if (found != m_chunks.end()) {
            for (const auto &block : found->second.completedBlocks) {
                completeBlock(block);
            }
        }
        m_retiredChunks.insert(key);
    }

    void PresentationTracker::completeDeferredBlocks() {
        for (const auto &block : m_deferredCompletedBlocks) {
            completeBlock(block);
        }
        m_deferredCompletedBlocks.clear();
    }

    std::optional<position_t> PresentationTracker::toolOffsetForChunk(
            const EpochId epoch, const ChunkId chunk) const {
        const auto found = m_chunks.find({epoch, chunk});
        if (found == m_chunks.end()) {
            return std::nullopt;
        }

        return found->second.active.tool.offset;
    }

    std::optional<PresentationExecutionSpanDiagnostic>
    PresentationTracker::executionSpanDiagnostic(
            const EpochId epoch, const SpanId span) const {
        const auto found = m_executionSpans.find({epoch, span});
        if (found == m_executionSpans.end()) {
            return std::nullopt;
        }

        return found->second;
    }
}
