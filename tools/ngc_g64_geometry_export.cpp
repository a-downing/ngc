#include <algorithm>
#include <array>
#include <atomic>
#include <expected>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/GeometryStreamProducer.h"
#include "machine/MachineConfiguration.h"
#include "machine/ToolTable.h"
#include "utils.h"

namespace {
    struct ExportChain {
        ngc::ContinuousChainId id = 0;
        std::vector<ngc::PreparedPathPiece> pieces;
        bool continuous = false;
        bool exactStop = false;
    };

    std::expected<ngc::spline_detail::SplineFitSolver, std::string> parseSmoother(std::string_view argument) {
        constexpr std::string_view prefix = "--smoother=";
        if (!argument.starts_with(prefix)) {
            return std::unexpected("smoother option must use --smoother=<name>");
        }
        argument.remove_prefix(prefix.size());

        using ngc::spline_detail::SplineFitSolver;
        if (argument == "none") {
            return SplineFitSolver::None;
        }
        if (argument == "coordinate") {
            return SplineFitSolver::CoordinateSearch;
        }
        if (argument == "uniform") {
            return SplineFitSolver::UniformBandedFairness;
        }
        if (argument == "peak-targeted") {
            return SplineFitSolver::PeakTargetedBandedFairness;
        }
        if (argument == "velocity-targeted") {
            return SplineFitSolver::VelocityTargetedBandedFairness;
        }

        return std::unexpected("unknown smoother; expected none, coordinate, uniform, peak-targeted, or velocity-targeted");
    }

    std::expected<std::string, std::string> loadSource(const std::filesystem::path &path) {
        auto source = ngc::readFile(path);
        if (!source) {
            return std::unexpected("failed to read " + path.string() + ": " + source.error().what());
        }

        return std::move(*source);
    }

    std::expected<std::vector<std::tuple<std::string, std::string>>, std::string> loadPrograms(const std::filesystem::path &program) {
        std::vector<std::filesystem::path> autoloadPaths;
        std::error_code error;
        std::filesystem::directory_iterator iterator("autoload", error);
        for (; !error && iterator != std::filesystem::directory_iterator(); iterator.increment(error)) {
            std::error_code typeError;
            if (iterator->is_regular_file(typeError)) {
                autoloadPaths.push_back(iterator->path());
            } else if (typeError) {
                return std::unexpected("failed to inspect autoload source: " + typeError.message());
            }
        }

        if (error) {
            return std::unexpected("failed to read autoload directory: " + error.message());
        }

        std::ranges::sort(autoloadPaths);

        std::vector<std::tuple<std::string, std::string>> programs;
        programs.reserve(autoloadPaths.size() + 1);
        for (const auto &path : autoloadPaths) {
            auto source = loadSource(path);
            if (!source) {
                return std::unexpected(source.error());
            }

            programs.emplace_back(std::move(*source), path.string());
        }

        auto source = loadSource(program);
        if (!source) {
            return std::unexpected(source.error());
        }

        programs.emplace_back(std::move(*source), program.string());

        return programs;
    }

    void writePosition(std::ostream &output, const std::string_view label, const ngc::position_t &position) {
        output << label << ' ' << position.x << ' ' << position.y << ' ' << position.z << '\n';
    }

    void writeCurve(std::ostream &output, const ngc::PreparedCurve &curve) {
        std::visit([&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::same_as<T, ngc::PreparedLineCurve>) {
                output << "curve line\n";
                writePosition(output, "from", value.from);
                writePosition(output, "to", value.to);
            } else if constexpr (std::same_as<T, ngc::PreparedArcCurve>) {
                output << "curve arc\n";
                writePosition(output, "from", value.arc.from());
                writePosition(output, "to", value.arc.to());
                output << "center " << value.arc.center().x << ' ' << value.arc.center().y << ' '
                       << value.arc.center().z << '\n';
                output << "axis " << value.arc.axis().x << ' ' << value.arc.axis().y << ' '
                       << value.arc.axis().z << '\n';
            } else {
                output << "curve bspline\n";
                output << "degree " << value.degree << '\n';
                output << "control_count " << value.controls.size() << '\n';
                for (const auto &control : value.controls) {
                    writePosition(output, "control", control);
                }
                output << "knot_count " << value.knots.size() << '\n';
                for (const auto knot : value.knots) {
                    output << "knot " << knot << '\n';
                }
            }
        }, curve.value);
    }

    std::expected<void, std::string> writeChain(const std::filesystem::path &path, const ExportChain &chain, const ngc::Machine::Unit unit) {
        std::ofstream output(path, std::ios::trunc);
        if (!output) {
            return std::unexpected("failed to open output file " + path.string());
        }

        output << std::setprecision(std::numeric_limits<double>::max_digits10);
        output << "ngc_g64_geometry 1\n";
        output << "units " << (unit == ngc::Machine::Unit::Inch ? "inch" : "millimeter") << '\n';
        output << "curve_count " << chain.pieces.size() << '\n';

        for (const auto &piece : chain.pieces) {
            output << "curve_interval " << piece.curveFrom << ' ' << piece.curveTo << '\n';
            if (piece.splineKnotIntervals.empty()) {
                output << "feed_count 1\n";
                output << "feed " << piece.programmedFeed << '\n';
            } else {
                output << "feed_count " << piece.splineKnotIntervals.size() << '\n';
                for (const auto &interval : piece.splineKnotIntervals) {
                    output << "feed " << interval.programmedFeed << '\n';
                }
            }

            writeCurve(output, *piece.curve);
            output << "end_curve\n";
        }

        output << "end_geometry\n";
        output.close();

        if (!output) {
            return std::unexpected("failed while writing output file " + path.string());
        }

        return {};
    }

    std::filesystem::path outputPath(const std::filesystem::path &input, const std::size_t index) {
        return input.parent_path() / (input.stem().string() + "." + std::to_string(index) + ".txt");
    }
}

int main(const int argc, char **argv) {
    if (argc != 3) {
        std::println(stderr, "usage: ngc_g64_geometry_export --smoother=none|coordinate|uniform|peak-targeted|velocity-targeted program.ngc");

        return 2;
    }

    const auto smoother = parseSmoother(argv[1]);
    if (!smoother) {
        std::println(stderr, "{}", smoother.error());

        return 2;
    }

    const std::filesystem::path program = argv[2];
    auto configuration = ngc::loadMachineConfiguration("machine.toml");
    if (!configuration) {
        std::println(stderr, "machine configuration error: {}", configuration.error());

        return 1;
    }

    ngc::ToolTable tools;
    if (auto loaded = tools.load(); !loaded) {
        std::println(stderr, "tool table error: {}", loaded.error());

        return 1;
    }

    auto programs = loadPrograms(program);
    if (!programs) {
        std::println(stderr, "{}", programs.error());

        return 1;
    }

    ngc::InterpreterSession session(configuration->unit, ngc::InterpretationMode::Preview);
    session.machine().toolTable() = tools;
    session.setPrograms(*programs);
    session.compile([](const auto &callback) { callback(); });

    if (!session.compiled()) {
        std::println(stderr, "G-code compilation failed");

        return 1;
    }

    session.begin();

    ngc::PreparedGeometryForwardChannel forward;
    ngc::GeometryFeedbackChannel feedback;
    std::atomic<bool> cancelled = false;
    ngc::GeometryStreamPolicy policy;
    policy.splineFitSolver = *smoother;
    ngc::GeometryStreamProducer producer(session, forward, feedback, cancelled, policy);
    auto producerSucceeded = false;
    std::thread producerThread([&] { producerSucceeded = producer.run(1); });
    std::map<ngc::ContinuousChainId, ExportChain> activeChains;
    std::vector<ExportChain> exportChains;
    std::optional<ngc::ProbeMove> pendingProbe;
    std::optional<std::string> failure;
    auto complete = false;

    const auto sendFeedback = [&](ngc::GeometryFeedback value) {
        auto message = std::make_unique<const ngc::GeometryFeedback>(std::move(value));
        return feedback.waitPush(std::move(message), [&] { return cancelled.load(); });
    };

    while (!complete) {
        ngc::PreparedForwardMessage message;
        if (!forward.waitPop(message, [&] { return cancelled.load(); })) {
            break;
        }

        std::visit([&](auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::same_as<T, ngc::PreparedGeometrySlice>) {
                auto &chain = activeChains[value.chain];
                chain.id = value.chain;
                for (const auto &command : value.commands) {
                    chain.continuous |= command.metadata.pathMode == ngc::ExecutablePathMode::Continuous;
                    chain.exactStop |= command.metadata.pathMode == ngc::ExecutablePathMode::ExactStop;
                }

                chain.pieces.insert(chain.pieces.end(), std::make_move_iterator(value.pieces.begin()),
                                    std::make_move_iterator(value.pieces.end()));
            } else if constexpr (std::same_as<T, ngc::PreparedStandaloneCommand>) {
                if (const auto *probe = std::get_if<ngc::ProbeMove>(&value.command.command)) {
                    pendingProbe = *probe;
                }
            } else if constexpr (std::same_as<T, ngc::PreparedContinuousEnd>) {
                auto found = activeChains.find(value.chain);
                if (found == activeChains.end()) {
                    failure = "prepared geometry ended an unknown chain";
                } else {
                    if (found->second.continuous && found->second.exactStop) {
                        failure = "prepared geometry chain mixed continuous and exact-stop motion";
                    } else if (found->second.continuous) {
                        exportChains.push_back(std::move(found->second));
                    }

                    activeChains.erase(found);
                }
            } else if constexpr (std::same_as<T, ngc::PreparedSynchronizationFence>) {
                if (!sendFeedback(ngc::ReleaseSynchronization{value.epoch, value.fence})) {
                    failure = "could not release an interpreter synchronization fence";
                }
            } else if constexpr (std::same_as<T, ngc::PreparedProbeFence>) {
                if (!pendingProbe || pendingProbe->id() != value.commandId) {
                    failure = "probe fence has no preceding probe move";
                } else if (!sendFeedback(ngc::DeliverProbeResult{value.epoch, {
                               pendingProbe->id(), ngc::ProbeStatus::Triggered,
                               pendingProbe->target(), pendingProbe->target()}})) {
                    failure = "could not return the canonical preview probe result";
                } else {
                    pendingProbe.reset();
                }
            } else if constexpr (std::same_as<T, ngc::PreparedFailure>) {
                failure = value.error;
                complete = true;
            } else if constexpr (std::same_as<T, ngc::PreparedProgramEnd>) {
                complete = true;
            }
        }, *message);

        if (failure) {
            cancelled.store(true);
            forward.notifyAll();
            feedback.notifyAll();
            complete = true;
        }
    }

    if (producerThread.joinable()) {
        producerThread.join();
    }

    if (failure) {
        std::println(stderr, "geometry export failed: {}", *failure);

        return 1;
    }

    if (!producerSucceeded) {
        std::println(stderr, "geometry producer stopped before completing the program");

        return 1;
    }

    if (!activeChains.empty()) {
        std::println(stderr, "geometry producer left an unterminated chain");

        return 1;
    }

    if (exportChains.empty()) {
        std::println(stderr, "the program contains no executable G64 geometry chains");

        return 1;
    }

    for (std::size_t index = 0; index < exportChains.size(); ++index) {
        const auto path = outputPath(program, index + 1);
        if (auto written = writeChain(path, exportChains[index], configuration->unit); !written) {
            std::println(stderr, "{}", written.error());

            return 1;
        }

        std::println("wrote {} curves to {}", exportChains[index].pieces.size(), path.string());
    }

    return 0;
}
