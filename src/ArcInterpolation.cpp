#include "machine/ArcInterpolation.h"

#include <algorithm>
#include <cmath>
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

    ArcReference::ArcReference(const MoveArc &arc) : m_arc(arc), m_geometry(arcGeometry(arc)) {
        if(!m_geometry) return;
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

    double ArcReference::integratedLength(const double from, const double to) const {
        if(to <= from) return 0.0;
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
        const auto distance = std::clamp(requestedDistance, 0.0, m_length);
        if(distance == 0.0) return 0.0;
        if(distance == m_length) return 1.0;
        const auto upper = std::lower_bound(m_lengthNodes.begin(), m_lengthNodes.end(), distance,
            [](const LengthNode &node, const double value) { return node.distance < value; });
        const auto lower = upper-1;
        auto from = lower->parameter;
        auto to = upper->parameter;
        for(unsigned iteration = 0; iteration < 48; ++iteration) {
            const auto middle = std::midpoint(from, to);
            const auto middleDistance = lower->distance + integratedLength(lower->parameter, middle);
            if(middleDistance < distance) from = middle;
            else to = middle;
        }
        return std::midpoint(from, to);
    }

    position_t ArcReference::positionAtDistance(const double distance) const {
        return position(parameterAtDistance(distance));
    }

    position_t ArcReference::tangentAtDistance(const double distance) const {
        const auto value = derivative(parameterAtDistance(distance));
        const auto magnitude = value.length();
        return magnitude > 0.0 ? scaled(value, 1.0/magnitude) : position_t{};
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
