#include "machine/ExactStopTrajectoryPlanner.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <type_traits>
#include <vector>

#include <ruckig/ruckig.hpp>

namespace ngc {
    namespace {
        double dot(const vec3_t &a, const vec3_t &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
        vec3_t cross(const vec3_t &a, const vec3_t &b) {
            return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
        }
        vec3_t scale(const vec3_t &v, const double amount) { return { v.x*amount, v.y*amount, v.z*amount }; }
        vec3_t normalized(const vec3_t &v) { return scale(v, 1.0 / v.length()); }
        vec3_t rotate(const vec3_t &v, const double angle, const vec3_t &axis) {
            return scale(v, std::cos(angle)) + scale(cross(axis, v), std::sin(angle))
                + scale(axis, dot(axis, v) * (1.0 - std::cos(angle)));
        }
        position_t scaled(const position_t &value, const double amount) {
            return { value.x*amount, value.y*amount, value.z*amount,
                     value.a*amount, value.b*amount, value.c*amount };
        }
        position_t add(const position_t &a, const position_t &b) { return a + b; }
        position_t subtract(const position_t &a, const position_t &b) { return a - b; }

        struct PathSample { position_t position; position_t tangent; };
        struct TimeBoundary { double time; double distance; double velocity; };

        AxisPolynomialSpan hermite(const SpanId id, const PathSample &from, const PathSample &to,
                                   const double fromVelocity, const double toVelocity, const double duration) {
            const auto v0t = scaled(from.tangent, fromVelocity * duration);
            const auto v1t = scaled(to.tangent, toVelocity * duration);
            const auto delta = subtract(to.position, from.position);
            AxisPolynomialSpan result;
            result.id = id;
            result.duration = duration;
            result.inverseDuration = 1.0 / duration;
            result.inverseDurationSquared = result.inverseDuration * result.inverseDuration;
            result.inverseDurationCubed = result.inverseDurationSquared * result.inverseDuration;
            result.a = add(scaled(delta, -2.0), add(v0t, v1t));
            result.b = subtract(scaled(delta, 3.0), add(scaled(v0t, 2.0), v1t));
            result.c = v0t;
            result.d = from.position;
            result.end.position = to.position;
            result.end.velocity = scaled(to.tangent, toVelocity);
            const auto secondAtEnd = add(scaled(result.a, 6.0), scaled(result.b, 2.0));
            result.end.acceleration = scaled(secondAtEnd, result.inverseDurationSquared);
            return result;
        }

        double maximumLinearAcceleration(const AxisPolynomialSpan &span) {
            const auto at = [&](const double u) {
                const auto value = scaled(add(scaled(span.a, 6.0*u), scaled(span.b, 2.0)),
                                          span.inverseDurationSquared);
                return std::sqrt(value.x*value.x + value.y*value.y + value.z*value.z
                    + value.a*value.a + value.b*value.b + value.c*value.c);
            };
            return std::max(at(0.0), at(1.0));
        }

        double maximumLinearJerk(const AxisPolynomialSpan &span) {
            const auto value = scaled(span.a, 6.0 * span.inverseDurationCubed);
            return std::sqrt(value.x*value.x + value.y*value.y + value.z*value.z
                + value.a*value.a + value.b*value.b + value.c*value.c);
        }

        std::expected<std::vector<TimeBoundary>, std::string> timeLaw(
                const double length, const double requestedVelocity, const double acceleration, const double jerk) {
            if(std::isinf(acceleration)) {
                return std::vector<TimeBoundary> {
                    { 0.0, 0.0, requestedVelocity },
                    { length / requestedVelocity, length, requestedVelocity },
                };
            }
            ruckig::InputParameter<1> input;
            input.current_position = {0.0}; input.current_velocity = {0.0}; input.current_acceleration = {0.0};
            input.target_position = {length}; input.target_velocity = {0.0}; input.target_acceleration = {0.0};
            input.max_velocity = {requestedVelocity}; input.max_acceleration = {acceleration}; input.max_jerk = {jerk};
            ruckig::Ruckig<1> generator;
            ruckig::Trajectory<1> trajectory;
            if(generator.calculate(input, trajectory) != ruckig::Result::Working)
                return std::unexpected("Ruckig failed to calculate exact-stop timing");
            std::vector<double> times {0.0};
            for(const auto duration : trajectory.get_profiles().front().front().t) {
                const auto next = times.back() + duration;
                if(next > times.back() + 1e-12) times.push_back(next);
            }
            std::vector<TimeBoundary> result;
            for(const auto time : times) {
                double position, velocity, scalarAcceleration;
                trajectory.at_time(time, position, velocity, scalarAcceleration);
                result.push_back({time, position, velocity});
            }
            result.back() = {trajectory.get_duration(), length, 0.0};
            return result;
        }
    }

    ExactStopTrajectoryPlanner::ExactStopTrajectoryPlanner(TrajectoryLimits limits) : m_limits(limits) { }

    void ExactStopTrajectoryPlanner::reset(const EpochId epoch, const position_t &position) {
        m_epoch = epoch;
        m_nextChunk = 1;
        m_nextSpan = 1;
        m_previousBranch = 0;
        m_position = position;
    }

    std::expected<PlanChunk, std::string> ExactStopTrajectoryPlanner::compile(const MachineCommand &command) {
        if(m_limits.pathAcceleration <= 0.0 || m_limits.pathJerk <= 0.0
           || m_limits.rapidSpeed <= 0.0 || m_limits.arcChordTolerance <= 0.0)
            return std::unexpected("trajectory limits must be positive");

        PlanChunk chunk;
        chunk.epoch = m_epoch;
        chunk.id = m_nextChunk++;
        chunk.predecessorBranch = m_previousBranch;
        chunk.branch = chunk.id;

        auto appendMotion = [&](const double length, const double speedPerMinute,
                                const auto &sample, const std::size_t geometricSegments) -> std::optional<std::string> {
            if(length <= 1e-12) {
                auto hold = hermite(m_nextSpan++, sample(0.0), sample(length), 0.0, 0.0, 1e-6);
                if(!chunk.normalMotion.push(hold)) return "trajectory chunk span capacity exceeded";
                return std::nullopt;
            }
            const auto requestedVelocity = speedPerMinute / 60.0;
            if(requestedVelocity <= 0.0) return "motion speed must be positive";
            const auto timing = timeLaw(length, requestedVelocity, m_limits.pathAcceleration, m_limits.pathJerk);
            if(!timing) return timing.error();
            const auto &boundaries = *timing;
            for(std::size_t phase = 1; phase < boundaries.size(); ++phase) {
                const auto &begin = boundaries[phase - 1];
                const auto &end = boundaries[phase];
                if(end.time - begin.time <= 1e-12) continue;
                const auto fraction = (end.distance - begin.distance) / length;
                const auto subdivisions = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(geometricSegments * fraction)));
                for(std::size_t part = 0; part < subdivisions; ++part) {
                    const auto f0 = static_cast<double>(part) / subdivisions;
                    const auto f1 = static_cast<double>(part + 1) / subdivisions;
                    const auto distance0 = std::lerp(begin.distance, end.distance, f0);
                    const auto distance1 = std::lerp(begin.distance, end.distance, f1);
                    const auto velocity0 = std::lerp(begin.velocity, end.velocity, f0);
                    const auto velocity1 = std::lerp(begin.velocity, end.velocity, f1);
                    const auto duration = (end.time - begin.time) / subdivisions;
                    if(!chunk.normalMotion.push(hermite(m_nextSpan++, sample(distance0), sample(distance1),
                                                        velocity0, velocity1, duration)))
                        return "trajectory chunk span capacity exceeded";
                }
            }

            double maximumAcceleration = 0.0;
            double maximumJerk = 0.0;
            for(const auto &span : chunk.normalMotion) {
                maximumAcceleration = std::max(maximumAcceleration, maximumLinearAcceleration(span));
                maximumJerk = std::max(maximumJerk, maximumLinearJerk(span));
            }
            const auto scaleFactor = std::max({1.0,
                std::sqrt(maximumAcceleration / m_limits.pathAcceleration),
                std::cbrt(maximumJerk / m_limits.pathJerk)});
            if(scaleFactor > 1.0 + 1e-9) {
                for(auto &span : chunk.normalMotion) {
                    span.duration *= scaleFactor;
                    span.inverseDuration /= scaleFactor;
                    span.inverseDurationSquared = span.inverseDuration * span.inverseDuration;
                    span.inverseDurationCubed = span.inverseDurationSquared * span.inverseDuration;
                    span.end.velocity = scaled(span.end.velocity, 1.0 / scaleFactor);
                    span.end.acceleration = scaled(span.end.acceleration, 1.0 / (scaleFactor * scaleFactor));
                }
            }
            return std::nullopt;
        };

        auto error = std::visit([&](const auto &value) -> std::optional<std::string> {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, MoveLine> || std::same_as<T, ProbeMove>) {
                const auto source = [&] {
                    if constexpr(std::same_as<T, MoveLine>) return value.from();
                    else return value.from();
                }();
                const auto target = [&] {
                    if constexpr(std::same_as<T, MoveLine>) return value.to();
                    else return value.target();
                }();
                const auto delta = subtract(target, source);
                auto length = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z
                    + delta.a*delta.a + delta.b*delta.b + delta.c*delta.c);
                const auto tangent = length > 1e-12 ? scaled(delta, 1.0 / length) : position_t{};
                const auto sample = [&](const double distance) { return PathSample { add(source, scaled(tangent, distance)), tangent }; };
                const auto speed = [&] {
                    if constexpr(std::same_as<T, MoveLine>) return value.speed() < 0.0 ? m_limits.rapidSpeed : value.speed();
                    else return value.feed();
                }();
                if(auto result = appendMotion(length, speed, sample, 1)) return result;
                if constexpr(std::same_as<T, ProbeMove>) {
                    if(!chunk.events.push({ 0,
                                            ProbeEvent { value.id(), value.stopOnContact(), value.errorIfNotFound() } }))
                        return "trajectory chunk event capacity exceeded";
                }
                m_position = target;
            } else if constexpr(std::same_as<T, MoveArc>) {
                const auto source = value.from();
                const vec3_t start { source.x, source.y, source.z };
                const vec3_t end { value.to().x, value.to().y, value.to().z };
                if(value.axis().length() <= 1e-12) return "arc axis must be nonzero";
                const auto axis = normalized(value.axis());
                const auto startDelta = start - value.center();
                const auto endDelta = end - value.center();
                const auto startArm = startDelta - scale(axis, dot(startDelta, axis));
                const auto endArm = endDelta - scale(axis, dot(endDelta, axis));
                const auto radius = startArm.length();
                if(radius <= 1e-12) return "arc radius must be nonzero";
                auto sweep = std::atan2(dot(axis, cross(normalized(startArm), normalized(endArm))),
                                        dot(normalized(startArm), normalized(endArm)));
                if(sweep < 0.0) sweep += 2.0 * std::numbers::pi;
                if((startArm - endArm).length() < 1e-9) sweep = 2.0 * std::numbers::pi;
                const auto axial = scale(axis, dot(end - start, axis));
                const auto rotaryDelta = position_t { 0, 0, 0, value.to().a-source.a,
                                                       value.to().b-source.b, value.to().c-source.c };
                const auto xyzLength = std::hypot(radius*sweep, axial.length());
                const auto length = std::hypot(xyzLength, std::sqrt(rotaryDelta.a*rotaryDelta.a
                    + rotaryDelta.b*rotaryDelta.b + rotaryDelta.c*rotaryDelta.c));
                const auto halfAngle = std::acos(std::clamp(1.0 - m_limits.arcChordTolerance / radius, -1.0, 1.0));
                const auto segments = halfAngle > 0.0
                    ? std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(sweep / (2.0*halfAngle)))) : 1;
                const auto positionAt = [&](const double distance) {
                    const auto u = length > 0.0 ? std::clamp(distance / length, 0.0, 1.0) : 1.0;
                    const auto arm = rotate(startArm, sweep*u, axis);
                    const auto xyz = value.center() + arm + scale(axial, u);
                    auto result = add(source, scaled(subtract(value.to(), source), u));
                    result.x = xyz.x; result.y = xyz.y; result.z = xyz.z;
                    return result;
                };
                const auto sample = [&](const double distance) {
                    const auto h = std::max(length * 1e-6, 1e-9);
                    const auto before = positionAt(std::max(0.0, distance-h));
                    const auto after = positionAt(std::min(length, distance+h));
                    const auto denominator = std::min(length, distance+h) - std::max(0.0, distance-h);
                    return PathSample { positionAt(distance), scaled(subtract(after, before), 1.0/denominator) };
                };
                auto arcSpeed = value.speed();
                if(!std::isinf(m_limits.pathAcceleration) && length > 1e-12) {
                    const auto curvature = radius * (sweep / length) * (sweep / length);
                    if(curvature > 1e-15)
                        arcSpeed = std::min(arcSpeed, 60.0 * std::sqrt(m_limits.pathAcceleration / curvature));
                }
                if(auto result = appendMotion(length, arcSpeed, sample, segments)) return result;
                m_position = value.to();
            } else if constexpr(std::same_as<T, SpindleStart> || std::same_as<T, SpindleStop>) {
                const PathSample held { m_position, {} };
                if(!chunk.normalMotion.push(hermite(m_nextSpan++, held, held, 0.0, 0.0, 1e-6)))
                    return "trajectory chunk span capacity exceeded";
                SpindleEvent spindle;
                if constexpr(std::same_as<T, SpindleStart>) {
                    spindle = { true, value.direction(), value.speed() };
                }
                if(!chunk.events.push({ 0, spindle })) return "trajectory chunk event capacity exceeded";
            }
            return std::nullopt;
        }, command);
        if(error) return std::unexpected(std::move(*error));

        chunk.branchState = chunk.normalMotion[chunk.normalMotion.size - 1].end;
        chunk.branchState.velocity = {};
        chunk.branchState.acceleration = {};
        const PathSample held { chunk.branchState.position, {} };
        auto stop = hermite(m_nextSpan++, held, held, 0.0, 0.0, 1e-6);
        stop.end = chunk.branchState;
        chunk.stopTail.push(stop);
        chunk.stopState = chunk.branchState;
        m_previousBranch = chunk.branch;
        return chunk;
    }
}
