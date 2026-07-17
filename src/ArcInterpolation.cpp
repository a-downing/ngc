#include "machine/ArcInterpolation.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <functional>
#include <numeric>

namespace ngc::simulation_detail {
    namespace {
        double dot(const vec3_t &a, const vec3_t &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
        vec3_t cross(const vec3_t &a, const vec3_t &b) { return { a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x }; }
        vec3_t scale(const vec3_t &v, const double s) { return { v.x*s, v.y*s, v.z*s }; }
        vec3_t normalize(const vec3_t &v) { return scale(v, 1.0/v.length()); }
        vec3_t rotate(const vec3_t &v, const double angle, const vec3_t &axis) {
            return scale(v, std::cos(angle)) + scale(cross(axis, v), std::sin(angle))
                + scale(axis, dot(axis, v)*(1.0-std::cos(angle)));
        }
        position_t scaled(const position_t &value, const double amount) {
            return { value.x*amount, value.y*amount, value.z*amount,
                     value.a*amount, value.b*amount, value.c*amount };
        }
        double positionDot(const position_t &left, const position_t &right) {
            return left.x*right.x+left.y*right.y+left.z*right.z
                +left.a*right.a+left.b*right.b+left.c*right.c;
        }

        double adaptiveSimpson(const std::function<double(double)> &function,
                               const double from, const double to, const double tolerance,
                               const double fromValue, const double middleValue, const double toValue,
                               const double estimate, const unsigned depth) {
            const auto middle = std::midpoint(from, to);
            const auto leftMiddle = std::midpoint(from, middle);
            const auto rightMiddle = std::midpoint(middle, to);
            const auto leftMiddleValue = function(leftMiddle);
            const auto rightMiddleValue = function(rightMiddle);
            const auto left = (middle-from) * (fromValue + 4.0*leftMiddleValue + middleValue) / 6.0;
            const auto right = (to-middle) * (middleValue + 4.0*rightMiddleValue + toValue) / 6.0;
            const auto refined = left + right;
            if(depth == 20 || std::abs(refined-estimate) <= 15.0*tolerance)
                return refined + (refined-estimate) / 15.0;
            return adaptiveSimpson(function, from, middle, tolerance*0.5,
                                   fromValue, leftMiddleValue, middleValue, left, depth+1)
                + adaptiveSimpson(function, middle, to, tolerance*0.5,
                                  middleValue, rightMiddleValue, toValue, right, depth+1);
        }
    }

    ArcReference::ArcReference(const MoveArc &arc, ArcInverseDiagnostics *inverseDiagnostics)
        : m_arc(arc), m_geometry(arcGeometry(arc)), m_inverseDiagnostics(inverseDiagnostics) {
        if(!m_geometry) return;
        if(m_inverseDiagnostics) ++m_inverseDiagnostics->constructionIntegralEvaluations;
        const auto function = [this](const double parameter) { return speed(parameter); };
        const auto fromValue = function(0.0);
        const auto middleValue = function(0.5);
        const auto toValue = function(1.0);
        const auto estimate = (fromValue + 4.0*middleValue + toValue) / 6.0;
        m_length = adaptiveSimpson(function, 0.0, 1.0, 1e-12*std::max(1.0, estimate),
                                   fromValue, middleValue, toValue, estimate, 0);

        // This table only brackets inversion. Each selected interval is integrated
        // again below, so interpolation error here cannot alter path distance.
        constexpr std::size_t intervals = 64;
        m_lengthNodes.reserve(intervals+1);
        m_lengthNodes.push_back({0.0, 0.0});
        double cumulative = 0.0;
        for(std::size_t index = 1; index <= intervals; ++index) {
            const auto parameter = static_cast<double>(index) / intervals;
            cumulative += integratedLength(static_cast<double>(index-1) / intervals, parameter);
            m_lengthNodes.push_back({parameter, cumulative});
        }
        m_lengthNodes.back().distance = m_length;
    }

    position_t ArcReference::position(const double requestedParameter) const {
        const auto parameter = std::clamp(requestedParameter, 0.0, 1.0);
        if(parameter == 0.0) return m_arc.from();
        if(parameter == 1.0) return m_arc.to();
        if(!m_geometry) return mix(m_arc.from(), m_arc.to(), parameter);
        const auto &geometry = *m_geometry;
        const auto start = rotate(geometry.startArm, geometry.sweep*parameter, geometry.axisUnit);
        const auto end = rotate(geometry.endArm, -geometry.sweep*(1.0-parameter), geometry.axisUnit);
        const auto radial = scale(start, 1.0-parameter) + scale(end, parameter);
        const auto xyz = geometry.center + radial + scale(geometry.axial, parameter);
        auto result = mix(m_arc.from(), m_arc.to(), parameter);
        result.x=xyz.x; result.y=xyz.y; result.z=xyz.z;
        return result;
    }

    position_t ArcReference::derivative(const double requestedParameter) const {
        const auto parameter = std::clamp(requestedParameter, 0.0, 1.0);
        if(!m_geometry) return m_arc.to()-m_arc.from();
        const auto &geometry = *m_geometry;
        const auto start = rotate(geometry.startArm, geometry.sweep*parameter, geometry.axisUnit);
        const auto end = rotate(geometry.endArm, -geometry.sweep*(1.0-parameter), geometry.axisUnit);
        const auto startDerivative = scale(cross(geometry.axisUnit, start), geometry.sweep);
        const auto endDerivative = scale(cross(geometry.axisUnit, end), geometry.sweep);
        const auto radialDerivative = scale(start, -1.0) + scale(startDerivative, 1.0-parameter)
            + end + scale(endDerivative, parameter);
        const auto xyz = radialDerivative + geometry.axial;
        return { xyz.x, xyz.y, xyz.z,
                 m_arc.to().a-m_arc.from().a, m_arc.to().b-m_arc.from().b,
                 m_arc.to().c-m_arc.from().c };
    }

    double ArcReference::speed(const double parameter) const { return derivative(parameter).length(); }

    std::size_t ArcReference::inverseCacheIndex(const double distance) {
        auto bits = std::bit_cast<std::uint64_t>(distance);
        bits ^= bits >> 33;
        bits *= 0xff51afd7ed558ccdULL;
        bits ^= bits >> 33;
        return static_cast<std::size_t>(bits) & (INVERSE_CACHE_SIZE-1);
    }

    position_t ArcReference::secondDerivative(const double requestedParameter) const {
        const auto parameter = std::clamp(requestedParameter, 0.0, 1.0);
        if(!m_geometry) return {};
        const auto &geometry = *m_geometry;
        const auto start = rotate(geometry.startArm, geometry.sweep*parameter, geometry.axisUnit);
        const auto end = rotate(geometry.endArm, -geometry.sweep*(1.0-parameter), geometry.axisUnit);
        const auto startDerivative = scale(cross(geometry.axisUnit, start), geometry.sweep);
        const auto endDerivative = scale(cross(geometry.axisUnit, end), geometry.sweep);
        const auto startSecond = scale(
            cross(geometry.axisUnit, startDerivative), geometry.sweep);
        const auto endSecond = scale(
            cross(geometry.axisUnit, endDerivative), geometry.sweep);
        const auto xyz = scale(startDerivative,-2.0)+scale(startSecond,1.0-parameter)
            +scale(endDerivative,2.0)+scale(endSecond,parameter);
        return {xyz.x,xyz.y,xyz.z,0.0,0.0,0.0};
    }

    position_t ArcReference::thirdDerivative(const double requestedParameter) const {
        const auto parameter = std::clamp(requestedParameter, 0.0, 1.0);
        if(!m_geometry) return {};
        const auto &geometry = *m_geometry;
        const auto start = rotate(geometry.startArm, geometry.sweep*parameter,
                                  geometry.axisUnit);
        const auto end = rotate(geometry.endArm,
            -geometry.sweep*(1.0-parameter), geometry.axisUnit);
        const auto startDerivative = scale(
            cross(geometry.axisUnit, start), geometry.sweep);
        const auto endDerivative = scale(
            cross(geometry.axisUnit, end), geometry.sweep);
        const auto startSecond = scale(
            cross(geometry.axisUnit, startDerivative), geometry.sweep);
        const auto endSecond = scale(
            cross(geometry.axisUnit, endDerivative), geometry.sweep);
        const auto startThird = scale(
            cross(geometry.axisUnit, startSecond), geometry.sweep);
        const auto endThird = scale(
            cross(geometry.axisUnit, endSecond), geometry.sweep);
        const auto xyz = scale(startSecond,-3.0)+scale(startThird,1.0-parameter)
            +scale(endSecond,3.0)+scale(endThird,parameter);
        return {xyz.x,xyz.y,xyz.z,0.0,0.0,0.0};
    }

    double ArcReference::integratedLength(const double from, const double to,
                                          const bool inverseEvaluation) const {
        if(to <= from) return 0.0;
        if(m_inverseDiagnostics) {
            if(inverseEvaluation) ++m_inverseDiagnostics->inverseIntegralEvaluations;
            else ++m_inverseDiagnostics->constructionIntegralEvaluations;
        }
        const auto function = [this](const double parameter) { return speed(parameter); };
        const auto middle = std::midpoint(from, to);
        const auto fromValue = function(from);
        const auto middleValue = function(middle);
        const auto toValue = function(to);
        const auto estimate = (to-from) * (fromValue + 4.0*middleValue + toValue) / 6.0;
        return adaptiveSimpson(function, from, to, 1e-13*std::max(1.0, estimate),
                               fromValue, middleValue, toValue, estimate, 0);
    }

    double ArcReference::parameterAtDistance(const double requestedDistance) const {
        if(m_length <= 0.0) return 1.0;
        if(m_inverseDiagnostics) ++m_inverseDiagnostics->queries;
        const auto distance = std::clamp(requestedDistance, 0.0, m_length);
        if(distance == 0.0) {
            if(m_inverseDiagnostics) ++m_inverseDiagnostics->endpointQueries;
            return 0.0;
        }
        if(distance == m_length) {
            if(m_inverseDiagnostics) ++m_inverseDiagnostics->endpointQueries;
            return 1.0;
        }
        auto &cache = m_inverseCache[inverseCacheIndex(distance)];
        if(cache.valid && cache.distance == distance) {
            if(m_inverseDiagnostics) ++m_inverseDiagnostics->exactCacheHits;
            return cache.parameter;
        }
        const auto upper = std::lower_bound(m_lengthNodes.begin(), m_lengthNodes.end(), distance,
            [](const LengthNode &node, const double value) { return node.distance < value; });
        const auto lower = upper-1;
        auto from = lower->parameter;
        auto to = upper->parameter;
        auto parameter = std::lerp(from, to,
            (distance-lower->distance)/(upper->distance-lower->distance));
        const auto distanceTolerance = 1e-12*std::max(1.0, m_length);
        for(unsigned iteration = 0; iteration < 12; ++iteration) {
            const auto parameterDistance = lower->distance
                +integratedLength(lower->parameter, parameter, true);
            if(m_inverseDiagnostics) {
                ++m_inverseDiagnostics->newtonIterations;
                m_inverseDiagnostics->maximumNewtonIterations = std::max(
                    m_inverseDiagnostics->maximumNewtonIterations,
                    static_cast<std::size_t>(iteration+1));
            }
            const auto error = parameterDistance-distance;
            if(std::abs(error) <= distanceTolerance) {
                if(iteration == 0 && m_inverseDiagnostics)
                    ++m_inverseDiagnostics->seedConvergences;
                cache = {distance, parameter, true};
                return parameter;
            }
            if(error < 0.0) from = parameter;
            else to = parameter;
            const auto localSpeed = speed(parameter);
            const auto newton = localSpeed > 1e-15 ? parameter-error/localSpeed : parameter;
            if(newton > from && newton < to) parameter = newton;
            else {
                parameter = std::midpoint(from, to);
                if(m_inverseDiagnostics) ++m_inverseDiagnostics->safeguardedBisections;
            }
        }
        if(m_inverseDiagnostics) ++m_inverseDiagnostics->iterationLimitHits;
        cache = {distance, parameter, true};
        return parameter;
    }

    position_t ArcReference::positionAtDistance(const double distance) const {
        return position(parameterAtDistance(distance));
    }

    position_t ArcReference::tangentAtDistance(const double distance) const {
        const auto value = derivative(parameterAtDistance(distance));
        const auto magnitude = value.length();
        return magnitude > 0.0 ? scaled(value, 1.0/magnitude) : position_t{};
    }

    position_t ArcReference::curvatureAtDistance(const double distance) const {
        const auto parameter=parameterAtDistance(distance);
        const auto first=derivative(parameter);
        const auto second=secondDerivative(parameter);
        const auto speedSquared=first.x*first.x+first.y*first.y+first.z*first.z
            +first.a*first.a+first.b*first.b+first.c*first.c;
        if(speedSquared<=1e-30) return {};
        const auto firstSecond=first.x*second.x+first.y*second.y+first.z*second.z
            +first.a*second.a+first.b*second.b+first.c*second.c;
        return scaled(second,1.0/speedSquared)
            -scaled(first,firstSecond/(speedSquared*speedSquared));
    }

    position_t ArcReference::curvatureDerivativeAtDistance(const double distance) const {
        const auto parameter=parameterAtDistance(distance);
        const auto first=derivative(parameter);
        const auto second=secondDerivative(parameter);
        const auto third=thirdDerivative(parameter);
        const auto speed=first.length();
        if(speed<=1e-15) return {};
        const auto firstSecond=positionDot(first,second);
        const auto secondSquared=positionDot(second,second);
        const auto firstThird=positionDot(first,third);
        const auto inverseSpeed2=1.0/(speed*speed);
        const auto inverseSpeed4=inverseSpeed2*inverseSpeed2;
        const auto inverseSpeed6=inverseSpeed4*inverseSpeed2;
        const auto parameterDerivative=scaled(third,inverseSpeed2)
            +scaled(second,-3.0*firstSecond*inverseSpeed4)
            +scaled(first,-(secondSquared+firstThird)*inverseSpeed4)
            +scaled(first,4.0*firstSecond*firstSecond*inverseSpeed6);
        return scaled(parameterDerivative,1.0/speed);
    }

    double ArcReference::chordErrorBound(const double fromDistance, const double toDistance) const {
        if(!m_geometry) return 0.0;
        const auto from = parameterAtDistance(fromDistance);
        const auto to = parameterAtDistance(toDistance);
        const auto &geometry = *m_geometry;
        const auto radius0 = geometry.startArm.length();
        const auto radius1 = geometry.endArm.length();
        const auto radiusChange = radius1-radius0;
        const auto maximumRadius = std::max(std::lerp(radius0, radius1, from),
                                            std::lerp(radius0, radius1, to));
        const auto maximumSecondDerivative = std::hypot(2.0*radiusChange*geometry.sweep,
                                                         maximumRadius*geometry.sweep*geometry.sweep);
        const auto width = to-from;
        return maximumSecondDerivative * width*width / 8.0;
    }

    double ArcReference::curvatureAccelerationBound() const {
        if(!m_geometry) return 0.0;
        const auto &geometry = *m_geometry;
        const auto radius0 = geometry.startArm.length();
        const auto radius1 = geometry.endArm.length();
        const auto radiusChange = radius1-radius0;
        const auto maximumSecondDerivative = std::hypot(2.0*radiusChange*geometry.sweep,
            std::max(radius0, radius1)*geometry.sweep*geometry.sweep);
        const auto minimumSpeed = std::min(speed(0.0), speed(1.0));
        return minimumSpeed > 0.0 ? maximumSecondDerivative/(minimumSpeed*minimumSpeed) : 0.0;
    }

    position_t mix(const position_t &from, const position_t &to, const double t) {
        return { std::lerp(from.x,to.x,t), std::lerp(from.y,to.y,t), std::lerp(from.z,to.z,t),
                 std::lerp(from.a,to.a,t), std::lerp(from.b,to.b,t), std::lerp(from.c,to.c,t) };
    }
    double linearDistance(const position_t &from, const position_t &to) {
        return vec3_t { to.x-from.x, to.y-from.y, to.z-from.z }.length();
    }
    std::optional<ArcGeometry> arcGeometry(const MoveArc &arc) {
        const vec3_t start { arc.from().x, arc.from().y, arc.from().z };
        const vec3_t end { arc.to().x, arc.to().y, arc.to().z };
        if(arc.axis().length() == 0.0) return std::nullopt;
        const auto axis = normalize(arc.axis());
        const auto startDelta = start-arc.center();
        const auto endDelta = end-arc.center();
        const auto startArm = startDelta-scale(axis,dot(startDelta,axis));
        const auto endArm = endDelta-scale(axis,dot(endDelta,axis));
        if(startArm.length() == 0.0 || endArm.length() == 0.0) return std::nullopt;
        auto sweep = std::atan2(dot(axis,cross(normalize(startArm),normalize(endArm))), dot(normalize(startArm),normalize(endArm)));
        if(sweep < 0.0) sweep += 2.0*std::numbers::pi;
        if((startArm-endArm).length() < 1e-9) sweep = 2.0*std::numbers::pi;
        return ArcGeometry { arc.center(), axis, startArm, endArm, scale(axis,dot(end-start,axis)), sweep };
    }
    position_t interpolate(const MoveArc &arc, const double t) {
        const auto geometry = arcGeometry(arc);
        if(!geometry) return mix(arc.from(),arc.to(),t);
        if(t <= 0.0) return arc.from();
        if(t >= 1.0) return arc.to();
        const auto radial = scale(rotate(geometry->startArm,geometry->sweep*t,geometry->axisUnit),1.0-t)
            + scale(rotate(geometry->endArm,-geometry->sweep*(1.0-t),geometry->axisUnit),t);
        const auto xyz = geometry->center+radial+scale(geometry->axial,t);
        auto result = mix(arc.from(),arc.to(),t);
        result.x=xyz.x; result.y=xyz.y; result.z=xyz.z;
        return result;
    }
    double pathLength(const MoveArc &arc) {
        return ArcReference(arc).length();
    }
}
