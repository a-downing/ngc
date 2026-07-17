#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <concepts>
#include <format>
#include <memory>
#include <map>
#include <optional>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/OwningSpscChannel.h"
#include "machine/PreparedGeometry.h"

namespace ngc {
    inline constexpr std::size_t PREPARED_GEOMETRY_QUEUE_CAPACITY = 64;
    inline constexpr std::size_t GEOMETRY_FEEDBACK_QUEUE_CAPACITY = 16;

    using PreparedForwardMessage = std::unique_ptr<PreparedStreamMessage>;
    using PreparedFeedbackMessage = std::unique_ptr<const GeometryFeedback>;
    using PreparedGeometryForwardChannel =
        OwningSpscChannel<PreparedForwardMessage, PREPARED_GEOMETRY_QUEUE_CAPACITY>;
    using GeometryFeedbackChannel =
        OwningSpscChannel<PreparedFeedbackMessage, GEOMETRY_FEEDBACK_QUEUE_CAPACITY>;

    struct GeometryStreamPolicy {
        double publishNominalDuration = 0.25;
    };

    struct GeometryStreamDiagnostics {
        std::uint64_t messagesPublished = 0;
        std::uint64_t slicesPublished = 0;
        std::uint64_t standaloneCommandsPublished = 0;
        std::uint64_t continuousEndsPublished = 0;
        std::uint64_t producerBackpressureCount = 0;
        std::uint64_t consumerEmptyWaitCount = 0;
        std::size_t forwardQueueHighWater = 0;
        std::size_t maximumPreparedPieces = 0;
        double maximumSliceNominalDuration = 0.0;
        double preparedSeconds = 0.0;
        double preparationSeconds = 0.0;
        std::size_t retainedSourceHighWater = 0;
        GeometryPreparationDiagnostics pieces;
        std::string lastFailure;
    };

    // Owns all calls into the active interpreter and the feed-independent
    // geometry builder. It publishes only immutable NRT messages; the backend
    // is intentionally not visible from this class.
    class GeometryStreamProducer {
        InterpreterSession &m_session;
        PreparedGeometryForwardChannel &m_forward;
        GeometryFeedbackChannel &m_feedback;
        std::atomic<bool> &m_cancelled;
        GeometryStreamPolicy m_policy;
        GeometryStreamDiagnostics m_diagnostics;
        GeometryEpoch m_epoch = 0;
        GeometrySequence m_sequence = 1;
        PreparedCommandId m_nextCommand = 1;
        PreparedPieceId m_nextPiece = 1;
        ContinuousChainId m_nextChain = 1;
        SynchronizationFenceId m_nextFence = 1;
        std::optional<ProbeMove> m_pendingProbe;
        std::optional<SynchronizationFenceId> m_pendingSynchronization;
        std::vector<BlockExecution> m_activeBlocks;
        std::vector<PreparedCommandRecord> m_continuous;
        std::map<PreparedCommandId, PreparedCommandRecord> m_commandRecords;
        std::vector<PreparedPathPiece> m_pendingPieces;
        double m_pendingLength = 0.0;
        double m_pendingDuration = 0.0;
        std::set<PreparedCommandId> m_activatedCommands;
        bool m_haveLongAnchor = false;
        bool m_processedLongAnchor = false;
        std::optional<double> m_continuousScale;
        std::optional<TrajectoryCommandPresentation> m_continuousPresentation;
        ContinuousChainId m_chain = 0;

        static bool samePosition(const position_t &left, const position_t &right) {
            return (left - right).length() <= 1e-12;
        }

        static bool continuousMotion(const PreparedCommandRecord &record) {
            if(record.metadata.pathMode != ExecutablePathMode::Continuous) return false;
            return std::visit([](const auto &command) {
                using T = std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T, MoveLine>)
                    return command.speed() > 0.0 && !command.machineCoordinates();
                else if constexpr(std::same_as<T, MoveArc>) return command.speed() > 0.0;
                else return false;
            }, record.command);
        }

        static std::optional<position_t> commandStart(const MachineCommand &command) {
            return std::visit([](const auto &value) -> std::optional<position_t> {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, MoveLine> || std::same_as<T, MoveArc>) return value.from();
                else return std::nullopt;
            }, command);
        }

        static std::optional<position_t> commandEnd(const MachineCommand &command) {
            return std::visit([](const auto &value) -> std::optional<position_t> {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, MoveLine> || std::same_as<T, MoveArc>) return value.to();
                else return std::nullopt;
            }, command);
        }

        bool publish(PreparedStreamMessage message) {
            auto value = std::make_unique<PreparedStreamMessage>(std::move(message));
            if(!m_forward.waitPush(std::move(value), [&] { return m_cancelled.load(std::memory_order_acquire); }))
                return false;
            ++m_diagnostics.messagesPublished;
            m_diagnostics.forwardQueueHighWater = std::max(m_diagnostics.forwardQueueHighWater,
                m_forward.size());
            return true;
        }

        void tag(PreparedPathPiece &piece) {
            piece.id = m_nextPiece++;
        }

        static double sourceLength(const PreparedCommandRecord &record) {
            return std::visit([](const auto &command) {
                using T = std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T, MoveLine>)
                    return (command.to() - command.from()).length();
                else if constexpr(std::same_as<T, MoveArc>)
                    return simulation_detail::pathLength(command);
                else return 0.0;
            }, record.command);
        }

        bool isLongSource(const PreparedCommandRecord &record) const {
            return sourceLength(record) > 6.0 * m_continuousScale.value_or(0.001);
        }

        void appendPending(PreparedPathPiece piece) {
            tag(piece);
            m_pendingLength += piece.length();
            if(piece.programmedFeed > 0.0)
                m_pendingDuration += piece.length() / piece.programmedFeed;
            m_pendingPieces.push_back(std::move(piece));
        }

        void pruneCommandRecords() {
            std::set<PreparedCommandId> retained;
            for(const auto &command : m_continuous) retained.insert(command.id);
            for(const auto &piece : m_pendingPieces) {
                retained.insert(piece.primaryCommand);
                retained.insert(piece.activationCommands.begin(), piece.activationCommands.end());
            }
            std::erase_if(m_commandRecords, [&](const auto &entry) {
                return !retained.contains(entry.first);
            });
        }

        bool publishPending() {
            if(m_pendingPieces.empty()) return true;
            PreparedGeometrySlice slice;
            slice.epoch = m_epoch;
            slice.sequence = m_sequence++;
            slice.chain = m_chain;
            slice.pathLength = m_pendingLength;
            slice.nominalDuration = m_pendingDuration;
            slice.pieces = std::move(m_pendingPieces);

            std::set<PreparedCommandId> referenced;
            for(const auto &piece : slice.pieces) {
                referenced.insert(piece.primaryCommand);
                referenced.insert(piece.activationCommands.begin(), piece.activationCommands.end());
            }
            for(const auto id : referenced) {
                const auto found = m_commandRecords.find(id);
                if(found == m_commandRecords.end())
                    throw std::runtime_error(std::format(
                        "prepared geometry references missing command {}", id));
                auto copy = found->second;
                copy.presentationActivation = m_activatedCommands.insert(id).second;
                slice.commands.push_back(std::move(copy));
            }

            m_diagnostics.preparedSeconds += slice.nominalDuration;
            m_diagnostics.maximumSliceNominalDuration = std::max(
                m_diagnostics.maximumSliceNominalDuration, slice.nominalDuration);
            m_diagnostics.maximumPreparedPieces = std::max(
                m_diagnostics.maximumPreparedPieces, slice.pieces.size());
            if(!publish(std::move(slice))) return false;
            ++m_diagnostics.slicesPublished;
            m_pendingPieces.clear();
            m_pendingLength = 0.0;
            m_pendingDuration = 0.0;
            pruneCommandRecords();
            return true;
        }

        std::expected<PreparedContinuousGeometry, std::string> prepareWindow(
                const bool deferFinalRetainedSection = false) {
            const auto started = std::chrono::steady_clock::now();
            position_t start{};
            if(const auto value = commandStart(m_continuous.front().command)) start = *value;
            auto prepared = prepareContinuousGeometry(m_continuous,
                m_continuousScale.value_or(0.001), start,
                GeometryPreparationEffort{ .certifySourceTube = false,
                    .generateSamples = true,
                    .lengthTableIntervalsPerKnotSpan = 32 },
                ContinuousGeometryBoundaries{
                    .incomingReplacement = m_processedLongAnchor,
                    .deferFinalRetainedSection = deferFinalRetainedSection });
            m_diagnostics.retainedSourceHighWater = std::max(
                m_diagnostics.retainedSourceHighWater, m_continuous.size());
            m_diagnostics.preparationSeconds += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            return prepared;
        }

        bool prepareThroughLongAnchor(const PreparedCommandId nextAnchor) {
            auto prepared = prepareWindow(true);
            if(!prepared) {
                m_diagnostics.lastFailure = prepared.error();
                publish(PreparedFailure{m_epoch, m_sequence++, prepared.error()});
                m_cancelled.store(true, std::memory_order_release);
                return false;
            }

            if(prepared->pieces.empty())
                throw std::runtime_error("incremental geometry window produced no finalized pieces");
            const auto outgoing = prepared->pieces.size() - 1;
            if(prepared->pieces.back().kind != PreparedPieceKind::JunctionBlend
               && prepared->pieces.back().kind != PreparedPieceKind::ClusterSpline)
                throw std::runtime_error("incremental geometry window has no outgoing replacement before its anchor");

            for(std::size_t index = 0; index < outgoing; ++index)
                appendPending(std::move(prepared->pieces[index]));

            if(m_pendingDuration >= m_policy.publishNominalDuration && !publishPending())
                return false;
            appendPending(std::move(prepared->pieces[outgoing]));

            const auto retainedAnchor = std::ranges::find(m_continuous,
                nextAnchor, &PreparedCommandRecord::id);
            if(retainedAnchor == m_continuous.end())
                throw std::runtime_error("incremental geometry source window lost its next anchor command");
            m_continuous.erase(m_continuous.begin(), retainedAnchor);
            m_haveLongAnchor = true;
            m_processedLongAnchor = true;
            pruneCommandRecords();
            return true;
        }

        bool flushContinuous(const PreparedBoundaryReason reason, const bool publishEnd) {
            if(m_continuous.empty()) return true;
            auto prepared = prepareWindow();
            if(!prepared) {
                m_diagnostics.lastFailure = prepared.error();
                publish(PreparedFailure{m_epoch, m_sequence++, prepared.error()});
                m_cancelled.store(true, std::memory_order_release);
                return false;
            }
            for(auto &piece : prepared->pieces)
                appendPending(std::move(piece));
            if(!publishPending()) return false;
            m_continuous.clear();
            m_commandRecords.clear();
            m_activatedCommands.clear();
            m_haveLongAnchor = false;
            m_processedLongAnchor = false;
            if(publishEnd) {
                m_continuousScale.reset();
                m_continuousPresentation.reset();
            }
            if(!publishEnd) return true;
            if(!publish(PreparedContinuousEnd{m_epoch, m_sequence++, m_chain, reason})) return false;
            ++m_diagnostics.continuousEndsPublished;
            m_chain = 0;
            return true;
        }

        bool waitForFeedback(const auto predicate) {
            while(!m_cancelled.load(std::memory_order_acquire)) {
                PreparedFeedbackMessage feedback;
                if(!m_feedback.waitPop(feedback, [&] { return m_cancelled.load(std::memory_order_acquire); }))
                    return false;
                const auto &value = *feedback;
                if(predicate(value)) return true;
                m_diagnostics.lastFailure = "geometry feedback did not match the pending barrier";
                m_cancelled.store(true, std::memory_order_release);
                return false;
            }
            return false;
        }

        TrajectoryCommandPresentation capturePresentation() const {
            TrajectoryCommandPresentation presentation;
            presentation.tool = m_session.machine().toolGeometry();
            presentation.activeToolOffset = m_session.machine().toolOffset();
            presentation.workCoordinateSystem = WorkCoordinateSystem {
                std::string(name(*m_session.machine().state().modeCoordSys)),
                m_session.machine().workOffset() };
            presentation.modalGCodes = m_session.machine().activeModalGCodes();
            presentation.activeBlocks = m_activeBlocks;
            return presentation;
        }

        PreparedCommandRecord makeRecord(MachineCommand command) {
            PreparedCommandRecord result;
            result.id = m_nextCommand++;
            result.command = std::move(command);
            result.metadata.pathMode = m_session.machine().state().modePath == GCPath::G64
                ? ExecutablePathMode::Continuous : ExecutablePathMode::ExactStop;
            result.metadata.pathTolerance = m_session.machine().pathTolerance();
            result.presentation = capturePresentation();
            return result;
        }

        bool publishStandalone(PreparedCommandRecord record) {
            PreparedStandaloneCommand standalone;
            standalone.epoch = m_epoch;
            standalone.sequence = m_sequence++;
            standalone.command = std::move(record);
            standalone.displayGeometry = prepareDisplayCurve(standalone.command.command);
            if(!publish(std::move(standalone))) return false;
            ++m_diagnostics.standaloneCommandsPublished;
            return true;
        }

        bool processCommand(MachineCommand command) {
            auto record = makeRecord(std::move(command));
            if(continuousMotion(record)) {
                const auto scale = record.metadata.pathTolerance.value_or(0.001);
                const auto compatible = !m_continuous.empty()
                    &&m_continuousScale && std::abs(*m_continuousScale - scale) <= 0.0
                    &&m_continuousPresentation && sameProtectedTrajectoryPresentation(
                        *m_continuousPresentation, record.presentation)
                    &&commandEnd(m_continuous.back().command)
                    &&commandStart(record.command)
                    &&samePosition(*commandEnd(m_continuous.back().command),
                                   *commandStart(record.command));
                if(!compatible && !m_continuous.empty()
                   &&!flushContinuous(PreparedBoundaryReason::IncompatibleCommand, true)) return false;
                if(m_continuous.empty() && m_chain == 0) {
                    m_chain = m_nextChain++;
                    m_continuousScale = scale;
                    m_continuousPresentation = record.presentation;
                } else if(m_continuous.empty()) {
                    m_continuousScale = scale;
                    m_continuousPresentation = record.presentation;
                }
                m_commandRecords.insert_or_assign(record.id, record);
                const auto commandId = record.id;
                const auto longSource = isLongSource(record);
                m_continuous.push_back(std::move(record));
                m_diagnostics.retainedSourceHighWater = std::max(
                    m_diagnostics.retainedSourceHighWater, m_continuous.size());
                if(longSource) {
                    if(!m_haveLongAnchor) m_haveLongAnchor = true;
                    else if(!prepareThroughLongAnchor(commandId)) return false;
                }
                return true;
            }
            if(!flushContinuous(PreparedBoundaryReason::IncompatibleCommand, true)) return false;
            if(!publishStandalone(std::move(record))) return false;
            return true;
        }

    public:
        GeometryStreamProducer(InterpreterSession &session,
                               PreparedGeometryForwardChannel &forward,
                               GeometryFeedbackChannel &feedback,
                               std::atomic<bool> &cancelled,
                               GeometryStreamPolicy policy = {})
            : m_session(session), m_forward(forward), m_feedback(feedback),
              m_cancelled(cancelled), m_policy(policy) { }

        GeometryStreamProducer(const GeometryStreamProducer &) = delete;
        GeometryStreamProducer &operator=(const GeometryStreamProducer &) = delete;

        bool run(const GeometryEpoch epoch) {
            m_epoch = epoch;
            m_sequence = 1;
            try {
                for(;;) {
                    if(m_cancelled.load(std::memory_order_acquire)) return false;
                    const auto event = m_session.nextWithBlocks([](const auto &callback) { callback(); });
                    if(const auto *lifecycle = std::get_if<InterpreterBlockLifecycle>(&event)) {
                        if(lifecycle->phase == BlockLifecyclePhase::Entered)
                            m_activeBlocks.push_back(lifecycle->block);
                        else if(const auto found = std::ranges::find_if(m_activeBlocks,
                                [&](const auto &block) { return block.id == lifecycle->block.id; });
                                found != m_activeBlocks.end()) m_activeBlocks.erase(found);
                        if(!publish(PreparedBlockLifecycleMessage{m_epoch, m_sequence++, *lifecycle})) return false;
                    } else if(const auto *command = std::get_if<MachineCommand>(&event)) {
                        if(const auto *probe = std::get_if<ProbeMove>(command)) {
                            if(!flushContinuous(PreparedBoundaryReason::Probe, true)) return false;
                            if(!publishStandalone(makeRecord(*command))) return false;
                            if(!publish(PreparedProbeFence{m_epoch, m_sequence++, probe->id()})) return false;
                            m_pendingProbe = *probe;
                            if(!waitForFeedback([&](const GeometryFeedback &feedback) {
                                const auto *result = std::get_if<DeliverProbeResult>(&feedback);
                                if(!result || result->epoch != m_epoch || result->result.id != probe->id())
                                    return false;
                                m_session.provideProbeResult(result->result);
                                return true;
                            })) return false;
                        } else if(!processCommand(*command)) return false;
                    } else if(std::holds_alternative<InterpreterWaitingForSynchronization>(event)) {
                        if(!flushContinuous(PreparedBoundaryReason::Synchronization, true)) return false;
                        const auto fence = m_nextFence++;
                        if(!publish(PreparedSynchronizationFence{m_epoch, m_sequence++, fence})) return false;
                        m_pendingSynchronization = fence;
                        if(!waitForFeedback([&](const GeometryFeedback &feedback) {
                            const auto *release = std::get_if<ReleaseSynchronization>(&feedback);
                            return release && release->epoch == m_epoch && release->fence == fence;
                        })) return false;
                        m_session.provideSynchronization();
                        m_pendingSynchronization.reset();
                    } else if(const auto *error = std::get_if<InterpreterError>(&event)) {
                        m_diagnostics.lastFailure = error->message;
                        publish(PreparedFailure{m_epoch, m_sequence++, error->message});
                        return false;
                    } else if(std::holds_alternative<InterpreterCompleted>(event)) {
                        if(!flushContinuous(PreparedBoundaryReason::ProgramEnd, true)) return false;
                        if(!publish(PreparedProgramEnd{m_epoch, m_sequence++})) return false;
                        return true;
                    } else if(const auto *waiting = std::get_if<InterpreterWaitingForProbe>(&event)) {
                        m_diagnostics.lastFailure = std::format(
                            "probe barrier {} was not preceded by its probe command", waiting->commandId);
                        publish(PreparedFailure{m_epoch, m_sequence++, m_diagnostics.lastFailure});
                        return false;
                    }
                }
            } catch(const std::exception &error) {
                m_diagnostics.lastFailure = error.what();
                publish(PreparedFailure{m_epoch, m_sequence++, m_diagnostics.lastFailure});
                return false;
            }
        }

        void stop() { m_cancelled.store(true, std::memory_order_release); m_forward.notifyAll(); m_feedback.notifyAll(); }
        const GeometryStreamDiagnostics &diagnostics() const { return m_diagnostics; }
        GeometryStreamDiagnostics takeDiagnostics() { return std::move(m_diagnostics); }
    };
}
