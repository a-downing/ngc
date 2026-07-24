#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/PreparedGeometry.h"
#include "machine/MotionBackend.h"

namespace ngc {
    struct MachinePresentationSnapshot {
        TrajectoryCommandPresentation activePresentation{};
        bool spindleRunning = false;
        double spindleSpeed = 0.0;
        Direction spindleDirection = Direction::CW;
        std::vector<WorkCoordinateSystem> usedWorkCoordinateSystems;
        std::vector<BlockExecution> completedBlocks;
        std::unordered_map<std::string, std::vector<std::uint8_t>> completedLineFlags;
    };

    struct PresentationExecutionSpanDiagnostic {
        ChunkId chunk = 0;
        std::uint32_t ordinal = 0;
        bool stopTail = false;
        double duration = 0.0;
        double distance = 0.0;
        double startVelocity = 0.0;
        double endVelocity = 0.0;
        double startAcceleration = 0.0;
        double endAcceleration = 0.0;
    };

    class PresentationTracker {
        using ChunkKey = std::pair<EpochId, ChunkId>;
        using MarkerKey = std::pair<EpochId, ExecutionMarkerId>;
        using SpanKey = std::pair<EpochId, SpanId>;

        struct ChunkPresentation {
            TrajectoryCommandPresentation active{};
            std::vector<BlockExecution> completedBlocks;
            std::optional<MachineCommand> command;
        };

        MachinePresentationSnapshot m_snapshot;
        std::map<ChunkKey, ChunkPresentation> m_chunks;
        std::map<MarkerKey, ChunkPresentation> m_markerPresentations;
        std::map<SpanKey, PresentationExecutionSpanDiagnostic> m_executionSpans;
        std::set<ChunkKey> m_chunksWithMarkers;
        std::set<ChunkKey> m_indexedDiagnosticChunks;
        std::map<std::uint64_t, ChunkKey> m_blockChunks;
        std::vector<BlockExecution> m_deferredCompletedBlocks;
        std::set<ChunkKey> m_retiredChunks;
        std::vector<BlockExecution> m_interpretedBlocks;

        void completeBlock(const BlockExecution &block);
        void applyPresentation(const ChunkPresentation &presentation);

    public:
        void reset(const TrajectoryCommandPresentation &initial = {});
        void clearTracking();
        const MachinePresentationSnapshot &snapshot() const;
        void setActivePresentation(const TrajectoryCommandPresentation &presentation);
        void setActiveWorkCoordinateSystem(const WorkCoordinateSystem &coordinateSystem);
        void clearActiveTool();
        void observeLifecycle(const InterpreterBlockLifecycle &lifecycle);
        void observeCommand(const MachineCommand &command, const ExecutionItem &item,
                            const TrajectoryCommandPresentation &captured,
                            ExecutionMarkerId activationMarker);
        void observeChunkAccepted(const ChunkAccepted &accepted);
        void observeMarkerReached(const ExecutionMarkerReached &marker);
        void observeChunkRetired(const ChunkRetired &retired);
        void completeDeferredBlocks();
        std::optional<position_t> toolOffsetForChunk(EpochId epoch, ChunkId chunk) const;
        std::optional<PresentationExecutionSpanDiagnostic> executionSpanDiagnostic(
            EpochId epoch, SpanId span) const;
    };
}
