#pragma once

#include <condition_variable>
#include <format>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

#include "evaluator/InterpreterSession.h"
#include "machine/GeometryStreamProducer.h"
#include "machine/ToolpathRecorder.h"
#include "machine/PreparedGeometry.h"
#include "memory/Vars.h"

class Worker {
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;

    ngc::InterpreterSession m_session;
    ngc::ToolpathRecorder m_toolpath;
    ngc::PreparedPreviewScene m_preparedPreview;
    std::unordered_map<ngc::Var, double> m_parameterSnapshot;

    bool m_doJoin = false;
    bool m_doCompile = false;
    bool m_doExecute = false;
    bool m_busy = false;

public:
    explicit Worker(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch)
        : m_session(unit, ngc::InterpretationMode::Preview) {
        refreshParameterSnapshot();
        m_thread = std::thread(&Worker::work, this);
    }

    ~Worker() { join(); }

    Worker(const Worker &) = delete;
    Worker &operator=(const Worker &) = delete;

    double read(ngc::Var var) const {
        std::scoped_lock lock(m_mutex);
        return m_parameterSnapshot.at(var);
    }

    auto lock(const auto &callback) const {
        std::scoped_lock lock(m_mutex);
        return callback();
    }

    const ngc::Machine &machine() const { return m_session.machine(); }
    const ngc::ToolpathRecorder &toolpath() const { return m_toolpath; }
    const ngc::PreparedPreviewScene &preparedPreview() const { return m_preparedPreview; }

    bool compiled() const {
        std::scoped_lock lock(m_mutex);
        return m_session.compiled();
    }

    bool busy() const {
        std::scoped_lock lock(m_mutex);
        return m_busy;
    }

    bool setToolTable(const ngc::ToolTable &tools) {
        std::scoped_lock lock(m_mutex);

        if(m_busy) {
            return false;
        }

        m_session.machine().toolTable() = tools;
        return true;
    }

    std::vector<ngc::Parser::Error> parserErrors() const {
        std::scoped_lock lock(m_mutex);
        return m_session.parserErrors();
    }

    std::vector<std::string> blockMessages() const {
        std::scoped_lock lock(m_mutex);
        return m_session.blockMessages();
    }

    bool compile(const std::vector<std::tuple<std::string, std::string>> &programs) {
        std::scoped_lock lock(m_mutex);

        if(m_busy) {
            return false;
        }

        m_session.setPrograms(programs);
        m_doCompile = true;
        m_cv.notify_one();
        return true;
    }

    void join() {
        std::unique_lock lock(m_mutex);
        if(!m_thread.joinable()) return;
        m_doJoin = true;
        m_session.requestStop();
        m_cv.notify_one();
        lock.unlock();
        m_thread.join();
    }

    bool execute() {
        std::scoped_lock lock(m_mutex);
        
        if(m_busy) {
            return false;
        }

        m_doExecute = true;
        m_cv.notify_one();
        return true;
    }

    std::vector<ngc::InterpreterStatusMessage> statusMessages() const {
        std::scoped_lock lock(m_mutex);
        return m_session.statusMessages();
    }

    bool clearToolpath() {
        std::scoped_lock lock(m_mutex);
        if(m_busy) return false;
        m_toolpath.clear();
        const auto revision = m_preparedPreview.revision + 1;
        m_preparedPreview = {};
        m_preparedPreview.revision = revision;
        return true;
    }

private:
    void work() {
        for(;;) {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [&] { return m_doJoin || m_doCompile || m_doExecute; });

            m_busy = true;

            if(m_doJoin) {
                m_doJoin = false;
                return;
            }

            if(m_doCompile) {
                m_doCompile = false;
                lock.unlock();
                doCompile();
                continue;
            }

            if(m_doExecute) {
                m_doExecute = false;
                lock.unlock();
                doExecute();
                continue;
            }
        }
    }

    void doCompile() {
        m_session.compile([&](const auto &callback) {
            std::scoped_lock lock(m_mutex);
            callback();
        });

        std::scoped_lock lock(m_mutex);
        m_busy = false;
    }

    void doExecute() {
        {
            std::scoped_lock lock(m_mutex);
            m_session.begin();
            m_toolpath.clear();
        }

        ngc::ToolpathRecorder preview;
        ngc::PreviewGeometryCollector preparedPreview;
        ngc::PreparedGeometryForwardChannel forward;
        ngc::GeometryFeedbackChannel feedback;
        std::atomic<bool> cancelled = false;
        ngc::GeometryStreamProducer producer(m_session, forward, feedback, cancelled);
        std::thread geometryThread([&] { (void)producer.run(1); });
        std::optional<std::string> failure;
        const auto sendFeedback = [&](ngc::GeometryFeedback value) {
            auto message = std::make_unique<const ngc::GeometryFeedback>(std::move(value));
            return feedback.waitPush(std::move(message), [&] { return cancelled.load(); });
        };
        const auto record = [&](const ngc::PreparedCommandRecord &command) {
            if(!command.presentationActivation) return;
            auto canonical = command.command;
            preview.consume(std::move(canonical), command.presentation.activeToolOffset,
                command.presentation.workCoordinateSystem.value_or(ngc::WorkCoordinateSystem{}),
                command.metadata.pathMode == ngc::ExecutablePathMode::Continuous,
                command.metadata.pathTolerance);
        };

        auto complete = false;
        while(!complete) {
            ngc::PreparedForwardMessage message;
            if(!forward.waitPop(message, [&] { return cancelled.load(); })) break;
            auto retainForPreview = false;
            std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, ngc::PreparedGeometrySlice>) {
                    for(const auto &command : value.commands) record(command);
                    retainForPreview = true;
                } else if constexpr(std::same_as<T, ngc::PreparedStandaloneCommand>) {
                    record(value.command);
                    retainForPreview = true;
                } else if constexpr(std::same_as<T, ngc::PreparedSynchronizationFence>) {
                    if(!sendFeedback(ngc::ReleaseSynchronization{value.epoch, value.fence}))
                        failure = "preview could not release an interpreter synchronization fence";
                } else if constexpr(std::same_as<T, ngc::PreparedProbeFence>) {
                    const auto commands = preview.commands();
                    const auto probe = std::ranges::find_if(commands, [&](const auto &command) {
                        const auto *candidate = std::get_if<ngc::ProbeMove>(&command);
                        return candidate && candidate->id() == value.commandId;
                    });
                    if(probe == commands.end()) {
                        failure = std::format("preview probe fence {} has no preceding probe", value.commandId);
                    } else {
                        const auto &move = std::get<ngc::ProbeMove>(*probe);
                        if(!sendFeedback(ngc::DeliverProbeResult{value.epoch, {
                                move.id(), ngc::ProbeStatus::Triggered,
                                move.target(), move.target()}}))
                            failure = "preview could not return its canonical probe result";
                    }
                } else if constexpr(std::same_as<T, ngc::PreparedFailure>) {
                    failure = std::format("preview continuous geometry preparation failed: {}", value.error);
                    complete = true;
                } else if constexpr(std::same_as<T, ngc::PreparedProgramEnd>) {
                    complete = true;
                }
            }, *message);
            if(retainForPreview) preparedPreview.consume(std::move(*message));
            if(failure) {
                cancelled.store(true);
                forward.notifyAll();
                feedback.notifyAll();
            }
        }
        if(geometryThread.joinable()) geometryThread.join();
        if(failure) {
            std::scoped_lock lock(m_mutex);
            m_session.reportError(*failure);
        }

        std::scoped_lock lock(m_mutex);
        m_toolpath.replace(std::move(preview));
        auto completedPreview = preparedPreview.finish();
        completedPreview.revision = m_preparedPreview.revision + 1;
        m_preparedPreview = std::move(completedPreview);
        refreshParameterSnapshot();
        m_busy = false;
    }

    void refreshParameterSnapshot() {
        for(const auto &[var, _name, _address, _flags, _value] : ngc::gVars) {
            m_parameterSnapshot.insert_or_assign(var, m_session.machine().memory().read(var));
        }
    }

    bool joinRequested() const {
        std::scoped_lock lock(m_mutex);
        return m_doJoin;
    }
};
