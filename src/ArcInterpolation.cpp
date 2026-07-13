#include "machine/ArcInterpolation.h"

#include <algorithm>
#include <cmath>
#include <numbers>

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
        const auto radial = scale(rotate(geometry->startArm,geometry->sweep*t,geometry->axisUnit),1.0-t)
            + scale(rotate(geometry->endArm,-geometry->sweep*(1.0-t),geometry->axisUnit),t);
        const auto xyz = geometry->center+radial+scale(geometry->axial,t);
        auto result = mix(arc.from(),arc.to(),t);
        result.x=xyz.x; result.y=xyz.y; result.z=xyz.z;
        return result;
    }
    double pathLength(const MoveArc &arc) {
        const auto geometry = arcGeometry(arc);
        if(!geometry) return (arc.to()-arc.from()).length();
        return std::hypot(0.5*(geometry->startArm.length()+geometry->endArm.length())*geometry->sweep,
                          geometry->axial.length());
    }
}
