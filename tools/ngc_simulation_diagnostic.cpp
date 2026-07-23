#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include "SimulationWorker.h"
#include "machine/MachineConfiguration.h"
#include "machine/ToolTable.h"
#include "utils.h"

namespace {
    const char *statusName(const ngc::SimulationStatus status) {
        switch(status) {
            case ngc::SimulationStatus::Stopped: return "Stopped";
            case ngc::SimulationStatus::Running: return "Running";
            case ngc::SimulationStatus::Holding: return "Holding";
            case ngc::SimulationStatus::Paused: return "Paused";
            case ngc::SimulationStatus::Completed: return "Completed";
            case ngc::SimulationStatus::Error: return "Error";
        }
        return "Unknown";
    }

    const char *backendName(const ngc::BackendState state) {
        switch(state) {
            case ngc::BackendState::Disabled: return "Disabled";
            case ngc::BackendState::Held: return "Held";
            case ngc::BackendState::Running: return "Running";
            case ngc::BackendState::Holding: return "Holding";
            case ngc::BackendState::Faulted: return "Faulted";
        }
        return "Unknown";
    }

    std::expected<std::string,std::string> load(const std::filesystem::path &path) {
        auto source=ngc::readFile(path);
        if(!source) return std::unexpected(
            "failed to read "+path.string()+": "+source.error().what());
        return std::move(*source);
    }

    const char *smootherName(const ngc::spline_detail::SplineFitSolver solver) {
        using ngc::spline_detail::SplineFitSolver;
        switch(solver) {
            case SplineFitSolver::None: return "none";
            case SplineFitSolver::CoordinateSearch: return "coordinate";
            case SplineFitSolver::UniformBandedFairness: return "uniform";
            case SplineFitSolver::PeakTargetedBandedFairness: return "peak-targeted";
            case SplineFitSolver::VelocityTargetedBandedFairness: return "velocity-targeted";
        }
        return "unknown";
    }

    std::expected<ngc::spline_detail::SplineFitSolver,std::string> parseSmoother(
            std::string_view argument) {
        constexpr std::string_view prefix="--smoother=";
        if(!argument.starts_with(prefix))
            return std::unexpected("smoother option must use --smoother=<name>");
        argument.remove_prefix(prefix.size());
        using ngc::spline_detail::SplineFitSolver;
        if(argument=="none") return SplineFitSolver::None;
        if(argument=="coordinate") return SplineFitSolver::CoordinateSearch;
        if(argument=="uniform") return SplineFitSolver::UniformBandedFairness;
        if(argument=="peak-targeted") return SplineFitSolver::PeakTargetedBandedFairness;
        if(argument=="velocity-targeted")
            return SplineFitSolver::VelocityTargetedBandedFairness;
        return std::unexpected(
            "unknown smoother; expected none, coordinate, uniform, peak-targeted, or "
            "velocity-targeted");
    }

    std::expected<ngc::ContinuousBoundaryAccelerationMode, std::string> parseMode(
            std::string_view argument) {
        constexpr std::string_view prefix = "--mode=";
        if (argument.starts_with(prefix)) {
            argument.remove_prefix(prefix.size());
        }
        if (argument == "zero") {
            return ngc::ContinuousBoundaryAccelerationMode::Zero;
        }
        if (argument == "optimized") {
            return ngc::ContinuousBoundaryAccelerationMode::Optimized;
        }

        return std::unexpected("unknown mode; expected zero or optimized");
    }

    std::string csvField(const std::string_view value) {
        std::string result{"\""};
        for (const auto character : value) {
            if (character == '"') {
                result += "\"\"";
            } else {
                result += character;
            }
        }
        result += '"';
        return result;
    }

    double component(const ngc::position_t &value, const std::size_t axis) {
        switch (axis) {
            case 0: return value.x;
            case 1: return value.y;
            case 2: return value.z;
            case 3: return value.a;
            case 4: return value.b;
            case 5: return value.c;
        }
        return 0.0;
    }

    std::string_view preparedKindName(const ngc::PreparedPieceKind kind) {
        switch (kind) {
            case ngc::PreparedPieceKind::RetainedLineSection: return "retained_line";
            case ngc::PreparedPieceKind::RetainedArcSection: return "retained_arc";
            case ngc::PreparedPieceKind::JunctionBlend: return "junction_blend";
            case ngc::PreparedPieceKind::ClusterSpline: return "cluster_spline";
        }
        return "unknown";
    }

    class QuinticFailureExporter {
        static constexpr std::array AXIS_NAMES{"X", "Y", "Z", "A", "B", "C"};

        std::filesystem::path m_prefix;
        std::ofstream m_failures;
        std::ofstream m_coefficients;
        std::ofstream m_sources;
        std::ofstream m_samples;
        std::ofstream m_shadowSpans;
        std::ofstream m_shadowActivations;
        std::ofstream m_shadowInputs;
        std::size_t m_plan = 0;
        std::size_t m_failure = 0;
        std::size_t m_shadowSpan = 0;
        std::size_t m_shadowActivation = 0;
        std::size_t m_verifiedShadowPlans = 0;
        std::size_t m_subServoJerkAccepted = 0;
        double m_maximumSubServoExcursionRatio = 0.0;
        double m_maximumAcceptedPointwiseRatio = 0.0;

        std::filesystem::path path(const std::string_view suffix) const {
            return std::filesystem::path{m_prefix.string() + std::string{suffix}};
        }

    public:
        explicit QuinticFailureExporter(std::filesystem::path prefix)
            : m_prefix(std::move(prefix)), m_failures(path("_failures.csv")),
              m_coefficients(path("_coefficients.csv")), m_sources(path("_sources.csv")),
              m_samples(path("_samples.csv")),
              m_shadowSpans(path("_shadow_spans.csv")),
              m_shadowActivations(path("_shadow_activations.csv")),
              m_shadowInputs(path("_shadow_inputs.csv")) {
            for (auto *stream : {&m_failures, &m_coefficients, &m_sources, &m_samples,
                    &m_shadowSpans, &m_shadowActivations, &m_shadowInputs}) {
                *stream << std::setprecision(std::numeric_limits<double>::max_digits10);
            }
            m_failures << "plan,failure,timing_piece,prepared_piece,prepared_kind,"
                "knot_interval,source_count,piece_curve_from,piece_curve_to,"
                "interval_from,interval_to,local_distance_from,local_distance_to,"
                "curve_distance_from,curve_distance_to,duration,certified_ratio,"
                "sampled_ratio,maximum_jerk_parameter,maximum_jerk_time,"
                "maximum_jerk_piece_time,maximum_jerk_ratio,"
                "maximum_jerk_x,maximum_jerk_y,maximum_jerk_z,"
                "maximum_jerk_a,maximum_jerk_b,maximum_jerk_c,"
                "original_distance,original_velocity,original_acceleration,"
                "original_scalar_jerk,original_jerk_ratio,"
                "original_jerk_x,original_jerk_y,original_jerk_z,"
                "original_jerk_a,original_jerk_b,original_jerk_c,constraint,axis\n";
            m_coefficients << "plan,failure,axis,"
                "bezier_b0,bezier_b1,bezier_b2,bezier_b3,bezier_b4,bezier_b5,"
                "normalized_c0,normalized_c1,normalized_c2,normalized_c3,"
                "normalized_c4,normalized_c5,"
                "seconds_c0,seconds_c1,seconds_c2,seconds_c3,seconds_c4,seconds_c5\n";
            m_sources << "plan,failure,source_ordinal,input,source,line,block_id,text\n";
            m_samples << "plan,failure,sample,parameter,time,local_distance,curve_distance,"
                "quintic_x,quintic_y,quintic_z,quintic_a,quintic_b,quintic_c,"
                "prepared_x,prepared_y,prepared_z,prepared_a,prepared_b,prepared_c,"
                "deviation\n";
            m_shadowSpans << "plan,shadow_span,timing_piece,prepared_piece,prepared_kind,"
                "knot_interval,first_source_input,last_source_input,source_count,degree,axis,"
                "global_time_from,piece_time_from,piece_time_to,local_distance_from,"
                "local_distance_to,duration,inverse_duration,origin,"
                "normalized_c0,normalized_c1,normalized_c2,normalized_c3,"
                "normalized_c4,normalized_c5,"
                "seconds_c0,seconds_c1,seconds_c2,seconds_c3,seconds_c4,seconds_c5,"
                "start_position,start_velocity,start_acceleration,"
                "end_position,end_velocity,end_acceleration,"
                "pointwise_constraint_ratio,acceptance_ratio,"
                "acceleration_excursion_ratio,subservo_jerk_accepted\n";
            m_shadowActivations << "plan,activation,input,shadow_span,global_time,"
                "local_distance,parameter,timing_piece,prepared_piece,"
                "source,line,block_id,text\n";
            m_shadowInputs << "plan,input,referenced_by_shadow,source,line,block_id,text\n";
        }

        bool good() const {
            return m_failures && m_coefficients && m_sources && m_samples
                && m_shadowSpans && m_shadowActivations && m_shadowInputs;
        }

        void observe(const ngc::ContinuousTrajectoryPlan &plan,
                     const std::span<const ngc::TrajectoryPlannerInput> inputs) {
            const auto &materialization = plan.materialization.quintic;
            m_subServoJerkAccepted += materialization.subServoJerkAcceptedSpans;
            m_maximumSubServoExcursionRatio = std::max(
                m_maximumSubServoExcursionRatio,
                materialization.maximumSubServoAccelerationExcursionRatio);
            m_maximumAcceptedPointwiseRatio = std::max(
                m_maximumAcceptedPointwiseRatio,
                materialization.maximumAcceptedPointwiseRatio);
            if (materialization.shadowSequenceVerified) {
                ++m_verifiedShadowPlans;
            }
            std::vector<bool> shadowReferencedInputs(inputs.size(), false);
            for (std::size_t spanIndex = 0;
                    spanIndex < materialization.shadowSpans.size(); ++spanIndex) {
                const auto &span = materialization.shadowSpans[spanIndex];
                const auto knot = span.knotInterval
                        == std::numeric_limits<std::size_t>::max()
                    ? std::string{"none"} : std::to_string(span.knotInterval);
                for (auto input = span.firstSourceInput;
                        input <= span.lastSourceInput && input < inputs.size(); ++input) {
                    shadowReferencedInputs[input] = true;
                }
                for (std::size_t axisIndex = 0;
                        axisIndex < AXIS_NAMES.size(); ++axisIndex) {
                    m_shadowSpans << m_plan << ',' << spanIndex << ','
                        << span.timingPiece << ',' << span.preparedPiece << ','
                        << preparedKindName(span.preparedKind) << ',' << knot << ','
                        << span.firstSourceInput << ',' << span.lastSourceInput << ','
                        << span.sourceInputCount << ',' << span.degree << ','
                        << AXIS_NAMES[axisIndex] << ',' << span.globalTimeFrom << ','
                        << span.pieceTimeFrom << ',' << span.pieceTimeTo << ','
                        << span.localDistanceFrom << ',' << span.localDistanceTo << ','
                        << span.duration << ',' << span.inverseDuration << ','
                        << component(span.origin, axisIndex);
                    for (const auto &coefficient : span.coefficients) {
                        m_shadowSpans << ',' << component(coefficient, axisIndex);
                    }
                    auto durationPower = 1.0;
                    for (const auto &coefficient : span.coefficients) {
                        m_shadowSpans << ','
                            << component(coefficient, axisIndex) / durationPower;
                        durationPower *= span.duration;
                    }
                    m_shadowSpans << ',' << component(span.start.position, axisIndex)
                        << ',' << component(span.start.velocity, axisIndex)
                        << ',' << component(span.start.acceleration, axisIndex)
                        << ',' << component(span.end.position, axisIndex)
                        << ',' << component(span.end.velocity, axisIndex)
                        << ',' << component(span.end.acceleration, axisIndex)
                        << ',' << span.pointwiseConstraintRatio
                        << ',' << span.acceptanceRatio
                        << ',' << span.accelerationExcursionRatio
                        << ',' << (span.subServoJerkAccepted ? 1 : 0) << '\n';
                }
                ++m_shadowSpan;
            }
            for (const auto &activation : materialization.shadowActivations) {
                const auto &span = materialization.shadowSpans[activation.span];
                std::string source;
                std::string text;
                std::size_t line = 0;
                std::uint64_t block = 0;
                if (activation.input < inputs.size()
                   && !inputs[activation.input].presentation.activeBlocks.empty()) {
                    const auto &active =
                        inputs[activation.input].presentation.activeBlocks.back();
                    source = active.source;
                    line = active.line;
                    block = active.id;
                    text = active.text;
                }
                m_shadowActivations << m_plan << ',' << m_shadowActivation++ << ','
                    << activation.input << ',' << activation.span << ','
                    << activation.globalTime << ',' << activation.localDistance << ','
                    << activation.parameter << ',' << span.timingPiece << ','
                    << span.preparedPiece << ',' << csvField(source) << ','
                    << line << ',' << block << ',' << csvField(text) << '\n';
            }
            for (std::size_t input = 0; input < inputs.size(); ++input) {
                if (!shadowReferencedInputs[input]) {
                    continue;
                }
                std::string source;
                std::string text;
                std::size_t line = 0;
                std::uint64_t block = 0;
                if (!inputs[input].presentation.activeBlocks.empty()) {
                    const auto &active =
                        inputs[input].presentation.activeBlocks.back();
                    source = active.source;
                    line = active.line;
                    block = active.id;
                    text = active.text;
                }
                m_shadowInputs << m_plan << ',' << input << ",1,"
                    << csvField(source) << ',' << line << ',' << block << ','
                    << csvField(text) << '\n';
            }
            for (const auto &failure : materialization.failures) {
                const auto failureIndex = m_failure++;
                const auto knot = failure.knotInterval
                        == std::numeric_limits<std::size_t>::max()
                    ? std::string{"none"} : std::to_string(failure.knotInterval);
                const auto axis = failure.axis < AXIS_NAMES.size()
                    ? AXIS_NAMES[failure.axis] : "none";
                m_failures << m_plan << ',' << failureIndex << ','
                    << failure.timingPiece << ',' << failure.preparedPiece << ','
                    << preparedKindName(failure.preparedKind) << ',' << knot << ','
                    << failure.sourceInputCount << ',' << failure.pieceCurveFrom << ','
                    << failure.pieceCurveTo << ',' << failure.intervalFrom << ','
                    << failure.intervalTo << ',' << failure.localDistanceFrom << ','
                    << failure.localDistanceTo << ','
                    << failure.pieceCurveFrom + failure.localDistanceFrom << ','
                    << failure.pieceCurveFrom + failure.localDistanceTo << ','
                    << failure.duration << ',' << failure.certifiedRatio << ','
                    << failure.sampledRatio << ',' << failure.maximumJerkParameter << ','
                    << failure.maximumJerkTime << ','
                    << failure.maximumJerkPieceTime << ','
                    << failure.maximumJerkRatio;
                for (std::size_t axisIndex = 0; axisIndex < AXIS_NAMES.size(); ++axisIndex) {
                    m_failures << ',' << component(failure.maximumJerk, axisIndex);
                }
                m_failures << ',' << failure.originalDistance << ','
                    << failure.originalVelocity << ','
                    << failure.originalAcceleration << ','
                    << failure.originalScalarJerk << ','
                    << failure.originalJerkRatio;
                for (std::size_t axisIndex = 0; axisIndex < AXIS_NAMES.size(); ++axisIndex) {
                    m_failures << ',' << component(failure.originalJerk, axisIndex);
                }
                m_failures << ',' << ngc::name(failure.constraint) << ',' << axis << '\n';

                for (std::size_t axisIndex = 0; axisIndex < AXIS_NAMES.size(); ++axisIndex) {
                    m_coefficients << m_plan << ',' << failureIndex << ','
                        << AXIS_NAMES[axisIndex];
                    for (const auto &control : failure.bezierControls) {
                        m_coefficients << ',' << component(control, axisIndex);
                    }
                    for (const auto &coefficient : failure.normalizedPowerCoefficients) {
                        m_coefficients << ',' << component(coefficient, axisIndex);
                    }
                    auto durationPower = 1.0;
                    for (const auto &coefficient : failure.normalizedPowerCoefficients) {
                        m_coefficients << ','
                            << component(coefficient, axisIndex) / durationPower;
                        durationPower *= failure.duration;
                    }
                    m_coefficients << '\n';
                }

                for (std::size_t ordinal = 0; ordinal < failure.sourceInputCount; ++ordinal) {
                    const auto input = failure.firstSourceInput + ordinal;
                    if (input >= inputs.size() || input > failure.lastSourceInput) {
                        break;
                    }
                    const auto &blocks = inputs[input].presentation.activeBlocks;
                    if (blocks.empty()) {
                        m_sources << m_plan << ',' << failureIndex << ',' << ordinal << ','
                            << input << ",\"\",0,0,\"\"\n";
                        continue;
                    }
                    const auto &block = blocks.back();
                    m_sources << m_plan << ',' << failureIndex << ',' << ordinal << ','
                        << input << ',' << csvField(block.source) << ',' << block.line << ','
                        << block.id << ',' << csvField(block.text) << '\n';
                }

                for (std::size_t sampleIndex = 0;
                        sampleIndex < failure.samples.size(); ++sampleIndex) {
                    const auto &sample = failure.samples[sampleIndex];
                    m_samples << m_plan << ',' << failureIndex << ',' << sampleIndex << ','
                        << sample.parameter << ',' << sample.time << ','
                        << sample.localDistance << ','
                        << failure.pieceCurveFrom + sample.localDistance;
                    for (std::size_t axisIndex = 0; axisIndex < AXIS_NAMES.size(); ++axisIndex) {
                        m_samples << ',' << component(sample.quinticPosition, axisIndex);
                    }
                    for (std::size_t axisIndex = 0; axisIndex < AXIS_NAMES.size(); ++axisIndex) {
                        m_samples << ',' << component(sample.preparedPosition, axisIndex);
                    }
                    m_samples << ',' << (sample.quinticPosition - sample.preparedPosition).length()
                        << '\n';
                }
            }
            ++m_plan;
        }

        std::string summary() const {
            return std::format(
                "failures={} subservo_jerk_accepted={} "
                "maximum_subservo_excursion_ratio={} "
                "maximum_accepted_pointwise_ratio={} shadow_spans={} "
                "shadow_activations={} verified_shadow_plans={} "
                "files=[{},{},{},{},{},{},{}]",
                m_failure, m_subServoJerkAccepted,
                m_maximumSubServoExcursionRatio,
                m_maximumAcceptedPointwiseRatio,
                m_shadowSpan, m_shadowActivation, m_verifiedShadowPlans,
                path("_failures.csv").string(), path("_coefficients.csv").string(),
                path("_sources.csv").string(), path("_samples.csv").string(),
                path("_shadow_spans.csv").string(),
                path("_shadow_activations.csv").string(),
                path("_shadow_inputs.csv").string());
        }
    };

    class TimeLawDistributionExporter {
        std::filesystem::path m_prefix;
        std::ofstream m_pieces;
        std::size_t m_plan = 0;
        std::size_t m_pieceCount = 0;

        std::filesystem::path path(const std::string_view suffix) const {
            return std::filesystem::path{m_prefix.string() + std::string{suffix}};
        }

    public:
        explicit TimeLawDistributionExporter(std::filesystem::path prefix)
            : m_prefix(std::move(prefix)), m_pieces(path("_pieces.csv")) {
            m_pieces << std::setprecision(
                std::numeric_limits<double>::max_digits10);
            m_pieces << "plan,timing_piece,input,prepared_piece,prepared_kind,knot_interval,"
                "length,duration,linear,curve_from,curve_to,programmed_velocity_limit,"
                "static_velocity_limit,velocity_limit,acceleration_limit,jerk_limit,"
                "entry_velocity,entry_acceleration,exit_velocity,exit_acceleration,"
                "source,line,block_id,text\n";
        }

        bool good() const {
            return static_cast<bool>(m_pieces);
        }

        void observe(const ngc::ContinuousTrajectoryPlan &plan,
                     const std::span<const ngc::TrajectoryPlannerInput> inputs) {
            for (std::size_t pieceIndex = 0;
                    pieceIndex < plan.pieceTiming.size(); ++pieceIndex) {
                const auto &piece = plan.pieceTiming[pieceIndex];
                const auto knot = piece.knotInterval
                        == std::numeric_limits<std::size_t>::max()
                    ? std::string{"none"} : std::to_string(piece.knotInterval);
                std::string source;
                std::string text;
                std::size_t line = 0;
                std::uint64_t block = 0;
                if (piece.input < inputs.size()
                   && !inputs[piece.input].presentation.activeBlocks.empty()) {
                    const auto &active =
                        inputs[piece.input].presentation.activeBlocks.back();
                    source = active.source;
                    line = active.line;
                    block = active.id;
                    text = active.text;
                }
                m_pieces << m_plan << ',' << pieceIndex << ',' << piece.input << ','
                    << piece.preparedPiece << ',' << preparedKindName(piece.preparedKind) << ','
                    << knot << ',' << piece.length << ',' << piece.duration << ','
                    << (piece.linear ? 1 : 0) << ',' << piece.curveFrom << ','
                    << piece.curveTo << ',' << piece.programmedVelocityLimit << ','
                    << piece.staticVelocityLimit << ',' << piece.velocityLimit << ','
                    << piece.accelerationLimit << ',' << piece.jerkLimit << ','
                    << piece.entryVelocity << ',' << piece.entryAcceleration << ','
                    << piece.exitVelocity << ',' << piece.exitAcceleration << ','
                    << csvField(source) << ',' << line << ',' << block << ','
                    << csvField(text) << '\n';
                ++m_pieceCount;
            }
            ++m_plan;
        }

        std::string summary() const {
            return std::format("pieces={} file={}", m_pieceCount,
                path("_pieces.csv").string());
        }
    };
}

int main(const int argc, char **argv) {
    auto smoother = ngc::spline_detail::continuousSplineFitSolver();
    auto mode = ngc::ContinuousBoundaryAccelerationMode::Optimized;
    std::optional<std::filesystem::path> quinticFailurePrefix;
    std::optional<std::filesystem::path> timeLawDistributionPrefix;
    std::optional<std::size_t> stopAfterPlanPieces;
    std::vector<std::string_view> positionalArguments;

    for (auto argument = 1; argument < argc; ++argument) {
        const auto option = std::string_view{argv[argument]};
        if (option.starts_with("--smoother=")) {
            const auto parsed = parseSmoother(option);
            if (!parsed) {
                std::println(stderr, "{}", parsed.error());
                return 2;
            }
            smoother = *parsed;
        } else if (option == "--mode" || option.starts_with("--mode=")) {
            if (option == "--mode" && ++argument == argc) {
                std::println(stderr, "mode option requires zero or optimized");
                return 2;
            }
            const auto parsed = parseMode(
                option == "--mode" ? std::string_view{argv[argument]} : option);
            if (!parsed) {
                std::println(stderr, "{}", parsed.error());
                return 2;
            }
            mode = *parsed;
        } else if (option.starts_with("--quintic-failures=")) {
            constexpr std::string_view prefix = "--quintic-failures=";
            const auto value = option.substr(prefix.size());
            if (value.empty()) {
                std::println(stderr, "quintic-failures option requires an output prefix");
                return 2;
            }
            quinticFailurePrefix = std::filesystem::path{value};
        } else if (option.starts_with("--time-law-distribution=")) {
            constexpr std::string_view prefix = "--time-law-distribution=";
            const auto value = option.substr(prefix.size());
            if (value.empty()) {
                std::println(stderr,
                    "time-law-distribution option requires an output prefix");
                return 2;
            }
            timeLawDistributionPrefix = std::filesystem::path{value};
        } else if (option.starts_with("--stop-after-plan-pieces=")) {
            constexpr std::string_view prefix = "--stop-after-plan-pieces=";
            const auto value = option.substr(prefix.size());
            std::size_t parsed = 0;
            const auto [end, error] =
                std::from_chars(value.data(), value.data() + value.size(), parsed);
            if (error != std::errc{} || end != value.data() + value.size()
               || parsed == 0) {
                std::println(stderr,
                    "stop-after-plan-pieces option requires a positive integer");
                return 2;
            }
            stopAfterPlanPieces = parsed;
        } else if (option.starts_with("--")) {
            std::println(stderr, "unknown diagnostic option: {}", option);
            return 2;
        } else {
            positionalArguments.push_back(option);
        }
    }

    if (positionalArguments.size() > 3) {
        std::println(stderr, "usage: ngc_simulation_diagnostic [program.ngc] "
            "[maximum-program-seconds] [tick-multiplier] "
            "[--smoother=none|coordinate|uniform|peak-targeted|velocity-targeted] "
            "[--mode=zero|optimized] "
            "[--quintic-failures=<output-prefix>] "
            "[--time-law-distribution=<output-prefix>] "
            "[--stop-after-plan-pieces=<count>]");
        return 2;
    }

    const std::filesystem::path program = positionalArguments.empty()
        ? std::filesystem::path{"adaptive_pockets.ngc"}
        : std::filesystem::path{positionalArguments[0]};
    const auto maximumProgramSeconds = positionalArguments.size() > 1
        ? std::strtod(positionalArguments[1].data(), nullptr) : 60.0;
    const auto multiplier = positionalArguments.size() > 2
        ? std::atoi(positionalArguments[2].data()) : 10;

    auto configuration=ngc::loadMachineConfiguration("machine.toml");
    if(!configuration) {
        std::println(stderr,"machine configuration error: {}",configuration.error());
        return 1;
    }
    ngc::ToolTable tools;
    if(auto loaded=tools.load();!loaded) {
        std::println(stderr,"tool table error: {}",loaded.error());
        return 1;
    }

    std::vector<std::tuple<std::string,std::string>> programs;
    for(const auto &path:std::array{
            std::filesystem::path{"autoload/hello.ngc"},
            std::filesystem::path{"autoload/world.ngc"},
            std::filesystem::path{"autoload/tool_change.ngc"},program}) {
        auto source=load(path);
        if(!source) {
            std::println(stderr,"{}",source.error());
            return 1;
        }
        programs.emplace_back(std::move(*source),path.string());
    }

    SimulationWorker worker(*configuration);
    worker.setTickMultiplier(multiplier);
    if(!worker.setSplineFitSolver(smoother)) {
        std::println(stderr,"simulation worker rejected the smoother selection");
        return 1;
    }
    auto planningEffort = ngc::ContinuousPlanningEffort{};
    planningEffort.boundaryAccelerationMode = mode;
    if (!worker.setContinuousPlanningEffort(planningEffort)) {
        std::println(stderr, "simulation worker rejected the continuous planning selection");
        return 1;
    }
    std::unique_ptr<QuinticFailureExporter> quinticFailureExporter;
    if (quinticFailurePrefix) {
        quinticFailureExporter =
            std::make_unique<QuinticFailureExporter>(*quinticFailurePrefix);
        if (!quinticFailureExporter->good()) {
            std::println(stderr, "failed to open quintic failure output files for prefix {}",
                quinticFailurePrefix->string());
            return 1;
        }
    }
    std::unique_ptr<TimeLawDistributionExporter> timeLawDistributionExporter;
    std::atomic<std::size_t> publishedPlanPieces = 0;
    if (timeLawDistributionPrefix) {
        timeLawDistributionExporter =
            std::make_unique<TimeLawDistributionExporter>(*timeLawDistributionPrefix);
        if (!timeLawDistributionExporter->good()) {
            std::println(stderr,
                "failed to open time-law distribution output files for prefix {}",
                timeLawDistributionPrefix->string());
            return 1;
        }
    }
    if (quinticFailureExporter || timeLawDistributionExporter) {
        if (!worker.setContinuousDiagnosticCallback(
                    [&](const ngc::ContinuousTrajectoryPlan &plan,
                        const std::span<const ngc::TrajectoryPlannerInput> inputs) {
                        if (quinticFailureExporter) {
                            quinticFailureExporter->observe(plan, inputs);
                        }
                        if (timeLawDistributionExporter) {
                            timeLawDistributionExporter->observe(plan, inputs);
                        }
                        publishedPlanPieces.store(plan.pieceTiming.size(),
                            std::memory_order_release);
                    })) {
            std::println(stderr, "simulation worker rejected the diagnostic callback");
            return 1;
        }
    }
    if(!worker.start(programs,tools,true)) {
        std::println(stderr,"simulation worker rejected the run");
        return 1;
    }

    std::println("program={} multiplier={} stop_after={:.3f}s smoother={} mode={} "
        "path_tempo_sampled_corrections=on ngc_dynamic_tolerance=1%",
        program.string(),multiplier,maximumProgramSeconds,smootherName(smoother),
        name(mode));
    ngc::EpochId lastEpoch=0;
    ngc::ChunkId lastChunk=0;
    ngc::SpanId lastSpan=0;
    std::string lastPlan;
    auto lastReport=std::chrono::steady_clock::now();
    auto reportedWorstWindow=0.0;
    for(;;) {
        const auto snapshot=worker.snapshot();
        if(snapshot.trajectoryPlanning.maximumContinuousHorizonSeconds
                >reportedWorstWindow+0.01) {
            reportedWorstWindow=
                snapshot.trajectoryPlanning.maximumContinuousHorizonSeconds;
            std::println("SLOW_WINDOW {:.6f}s driver={} plan={}",reportedWorstWindow,
                snapshot.trajectoryDriverActivity,
                snapshot.trajectoryContinuousPlanSummary);
        }
        if(snapshot.trajectoryContinuousPlanSummary!=lastPlan) {
            lastPlan=snapshot.trajectoryContinuousPlanSummary;
            if(!lastPlan.empty()) {
                std::println("PLAN {}",lastPlan);
                std::println("CORRECTIONS {}",
                    snapshot.trajectoryContinuousCorrectionHistory.empty()
                        ?std::string{"<none>"}
                        :snapshot.trajectoryContinuousCorrectionHistory);
            }
        }
        if(snapshot.trajectoryBackendEpoch!=lastEpoch
           ||snapshot.trajectoryBackendChunk!=lastChunk
           ||snapshot.trajectoryBackendSpan!=lastSpan) {
            lastEpoch=snapshot.trajectoryBackendEpoch;
            lastChunk=snapshot.trajectoryBackendChunk;
            lastSpan=snapshot.trajectoryBackendSpan;
            const auto block=snapshot.activeBlocks.empty()
                ?std::string{"<none>"}:snapshot.activeBlocks.back().text;
            std::println(
                "t={:.6f} backend={} epoch={} chunk={} span={} progress={:.6f} "
                "v={:.9g} a={:.9g} committed={:.6f}s active={:.6f}s queued={:.6f}s "
                "stop={:.6f}s queued_items={} xyz=[{:.9g},{:.9g},{:.9g}] block='{}'\n"
                "  span_detail={}\n  driver={}\n  latest_plan={}",
                snapshot.programElapsedSeconds,backendName(snapshot.trajectoryBackendState),
                lastEpoch,lastChunk,lastSpan,snapshot.trajectoryBackendSpanProgress,
                snapshot.trajectoryBackendVelocity,snapshot.trajectoryBackendAcceleration,
                snapshot.trajectoryBackendCommittedNormalSeconds,
                snapshot.trajectoryBackendActiveNormalRemainingSeconds,
                snapshot.trajectoryBackendQueuedNormalSeconds,
                snapshot.trajectoryBackendStopBranchSeconds,
                snapshot.trajectoryBackendQueuedExecutionItems,
                snapshot.machinePosition.x,snapshot.machinePosition.y,snapshot.machinePosition.z,
                block,snapshot.trajectoryBackendSpanDetail,
                snapshot.trajectoryDriverActivity,
                snapshot.trajectoryContinuousPlanSummary);
        }
        const auto now=std::chrono::steady_clock::now();
        if(now-lastReport>=std::chrono::seconds(5)) {
            lastReport=now;
            std::println("progress t={:.3f}s ticks={} status={} committed={:.6f}s "
                "active={:.6f}s queued={:.6f}s queued_items={} driver={}",
                snapshot.programElapsedSeconds,snapshot.servoTicks,
                statusName(snapshot.status),snapshot.trajectoryBackendCommittedNormalSeconds,
                snapshot.trajectoryBackendActiveNormalRemainingSeconds,
                snapshot.trajectoryBackendQueuedNormalSeconds,
                snapshot.trajectoryBackendQueuedExecutionItems,
                snapshot.trajectoryDriverActivity);
        }
        if(snapshot.status==ngc::SimulationStatus::Completed
           ||snapshot.status==ngc::SimulationStatus::Error
           ||snapshot.programElapsedSeconds>=maximumProgramSeconds
           ||(stopAfterPlanPieces
              &&publishedPlanPieces.load(std::memory_order_acquire)
                    >=*stopAfterPlanPieces)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    worker.stop();
    worker.join();
    const auto finalSnapshot=worker.snapshot();
    const auto geometrySeconds=finalSnapshot.geometryStream.preparationSeconds;
    const auto plannerSeconds=finalSnapshot.trajectoryPlanning.totalPlanningSeconds;
    const auto averageWindowSeconds=finalSnapshot.trajectoryPlanning.continuousHorizons==0
        ?0.0:finalSnapshot.trajectoryPlanning.totalContinuousHorizonSeconds
            /static_cast<double>(finalSnapshot.trajectoryPlanning.continuousHorizons);
    std::println("final status={} t={:.6f}s planned={:.6f}s "
        "geometry_processing={:.6f}s planner_processing={:.6f}s "
        "best_window={:.6f}s average_window={:.6f}s worst_window={:.6f}s "
        "total_processing={:.6f}s ticks={} error='{}'",
        statusName(finalSnapshot.status),finalSnapshot.programElapsedSeconds,
        finalSnapshot.trajectoryPlanning.plannedDuration,geometrySeconds,plannerSeconds,
        finalSnapshot.trajectoryPlanning.minimumContinuousHorizonSeconds,
        averageWindowSeconds,
        finalSnapshot.trajectoryPlanning.maximumContinuousHorizonSeconds,
        geometrySeconds+plannerSeconds,finalSnapshot.servoTicks,finalSnapshot.error);
    std::println("rolling candidates={} suffix_failures={} prefix_failures={} "
        "max_suffix_pieces={} rolling_search={:.6f}s",
        finalSnapshot.trajectoryPlanning.rollingBoundaryCandidates,
        finalSnapshot.trajectoryPlanning.rollingSuffixProbeFailures,
        finalSnapshot.trajectoryPlanning.rollingPrefixProbeFailures,
        finalSnapshot.trajectoryPlanning.maximumRollingSuffixProbePieces,
        finalSnapshot.trajectoryPlanning.rollingSearchSeconds);
    if (quinticFailureExporter) {
        std::println("quintic_failure_export {}", quinticFailureExporter->summary());
    }
    if (timeLawDistributionExporter) {
        std::println("time_law_distribution_export {}",
            timeLawDistributionExporter->summary());
    }
    return 0;
}
