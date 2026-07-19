#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace ngc::spline_detail {
    using Vector3 = std::array<double,3>;

    struct Endpoint3 {
        Vector3 position{};
        Vector3 tangent{};
        Vector3 curvature{};
    };

    inline Vector3 add(const Vector3 &a,const Vector3 &b);
    inline Vector3 subtract(const Vector3 &a,const Vector3 &b);
    inline Vector3 scaled(const Vector3 &value,double amount);
    inline double dot(const Vector3 &a,const Vector3 &b);
    inline double length(const Vector3 &value);
    inline Vector3 cross(const Vector3 &a,const Vector3 &b);

    struct ShortEntitySplineCluster {
        std::size_t left=0;
        std::size_t right=0;
    };

    inline std::vector<ShortEntitySplineCluster> detectShortEntitySplineClusters(
            const std::span<const double> lengths,const double programmedScale) {
        std::vector<ShortEntitySplineCluster> result;
        if(!std::isfinite(programmedScale)||programmedScale<=0.0||lengths.size()<3)
            return result;
        const auto threshold=6.0*programmedScale;
        for(std::size_t left=0;left+2<lengths.size();) {
            if(lengths[left]<=threshold) {
                ++left;
                continue;
            }
            auto right=left+1;
            while(right<lengths.size()&&lengths[right]<=threshold) ++right;
            if(right>left+1&&right<lengths.size()&&lengths[right]>threshold) {
                result.push_back({left,right});
                left=right;
            } else ++left;
        }
        return result;
    }

    inline Vector3 add(const Vector3 &a,const Vector3 &b) {
        return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};
    }
    inline Vector3 subtract(const Vector3 &a,const Vector3 &b) {
        return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};
    }
    inline Vector3 scaled(const Vector3 &value,const double amount) {
        return {value[0]*amount,value[1]*amount,value[2]*amount};
    }
    inline double dot(const Vector3 &a,const Vector3 &b) {
        return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    }
    inline double length(const Vector3 &value) { return std::sqrt(dot(value,value)); }
    inline Vector3 cross(const Vector3 &a,const Vector3 &b) {
        return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
    }

    inline std::optional<Vector3> inferShortLineMidpointCurvature(
            Vector3 previousTangent,Vector3 tangent,Vector3 nextTangent,
            const double lineLength) {
        const auto normalize=[](Vector3 value) -> std::optional<Vector3> {
            const auto magnitude=length(value);
            if(!std::isfinite(magnitude)||magnitude<=1e-12) return std::nullopt;
            return scaled(value,1.0/magnitude);
        };
        const auto previous=normalize(previousTangent);
        const auto current=normalize(tangent);
        const auto next=normalize(nextTangent);
        if(!previous||!current||!next||!std::isfinite(lineLength)||lineLength<=1e-12)
            return std::nullopt;
        const auto incomingCross=cross(*previous,*current);
        const auto outgoingCross=cross(*current,*next);
        const auto incomingSine=length(incomingCross);
        const auto outgoingSine=length(outgoingCross);
        const auto incomingAngle=std::atan2(incomingSine,
            std::clamp(dot(*previous,*current),-1.0,1.0));
        const auto outgoingAngle=std::atan2(outgoingSine,
            std::clamp(dot(*current,*next),-1.0,1.0));
        constexpr double MINIMUM_TURN=1e-5;
        constexpr double MAXIMUM_TURN=1.0;
        if(incomingAngle<MINIMUM_TURN||outgoingAngle<MINIMUM_TURN
           ||incomingAngle>MAXIMUM_TURN||outgoingAngle>MAXIMUM_TURN)
            return std::nullopt;
        const auto minimumAngle=std::min(incomingAngle,outgoingAngle);
        if(std::max(incomingAngle,outgoingAngle)>3.0*minimumAngle) return std::nullopt;
        const auto incomingBinormal=scaled(incomingCross,1.0/incomingSine);
        const auto outgoingBinormal=scaled(outgoingCross,1.0/outgoingSine);
        if(dot(incomingBinormal,outgoingBinormal)<0.9) return std::nullopt;
        const auto binormal=normalize(add(incomingBinormal,outgoingBinormal));
        if(!binormal) return std::nullopt;
        const auto deflection=0.5*(incomingAngle+outgoingAngle);
        const auto radius=0.5*lineLength/std::tan(0.5*deflection);
        if(!std::isfinite(radius)||radius<=1e-12) return std::nullopt;
        const auto normal=normalize(cross(*binormal,*current));
        if(!normal) return std::nullopt;
        return scaled(*normal,1.0/radius);
    }

    template<std::size_t ControlCount,std::size_t KnotCount>
    inline Vector3 evaluate(const std::array<Vector3,ControlCount> &controls,
                            const std::array<double,KnotCount> &knots,
                            const std::size_t degree,const double requestedParameter) {
        const auto endParameter=knots[ControlCount];
        if(requestedParameter<=knots[degree]) return controls.front();
        if(requestedParameter>=endParameter) return controls.back();
        const auto parameter=std::clamp(requestedParameter,knots[degree],endParameter);
        auto span=degree;
        while(span+1<ControlCount&&parameter>=knots[span+1]) ++span;
        std::array<Vector3,ControlCount> work{};
        for(std::size_t index=0;index<=degree;++index)
            work[index]=controls[span-degree+index];
        for(std::size_t level=1;level<=degree;++level)
            for(std::size_t index=degree;index>=level;--index) {
                const auto knotIndex=span-degree+index;
                const auto denominator=knots[knotIndex+degree-level+1]-knots[knotIndex];
                const auto alpha=(parameter-knots[knotIndex])/denominator;
                work[index]=add(scaled(work[index-1],1.0-alpha),scaled(work[index],alpha));
            }
        return work[degree];
    }

    inline std::pair<double,double> optimizeHandles(
            const Endpoint3 &start,const Endpoint3 &end,
            const double incomingTangentDistance,const double outgoingTangentDistance,
            const double initialIncoming,const double initialOutgoing,
            const double incomingScale,const double outgoingScale) {
        constexpr std::array<double,10> KNOTS{0,0,0,0,1,2,3,3,3,3};
        constexpr std::array<double,8> FIRST_KNOTS{0,0,0,1,2,3,3,3};
        constexpr std::array<double,6> SECOND_KNOTS{0,0,1,2,3,3};
        const auto controlsFor=[&](const double incoming,const double outgoing) {
            return std::array<Vector3,6>{
                start.position,
                add(start.position,scaled(start.tangent,incoming)),
                add(add(start.position,scaled(start.tangent,incomingTangentDistance)),
                    scaled(start.curvature,3.0*incoming*incoming)),
                add(add(end.position,scaled(end.tangent,outgoingTangentDistance)),
                    scaled(end.curvature,3.0*outgoing*outgoing)),
                subtract(end.position,scaled(end.tangent,outgoing)),
                end.position,
            };
        };
        const auto score=[&](const double incoming,const double outgoing) {
            const auto controls=controlsFor(incoming,outgoing);
            std::array<Vector3,5> firstControls{};
            std::array<Vector3,4> secondControls{};
            std::array<Vector3,3> thirdControls{};
            for(std::size_t i=0;i<firstControls.size();++i)
                firstControls[i]=scaled(subtract(controls[i+1],controls[i]),
                    3.0/(KNOTS[i+4]-KNOTS[i+1]));
            for(std::size_t i=0;i<secondControls.size();++i)
                secondControls[i]=scaled(subtract(firstControls[i+1],firstControls[i]),
                    2.0/(FIRST_KNOTS[i+3]-FIRST_KNOTS[i+1]));
            for(std::size_t i=0;i<thirdControls.size();++i)
                thirdControls[i]=scaled(subtract(secondControls[i+1],secondControls[i]),
                    1.0/(SECOND_KNOTS[i+2]-SECOND_KNOTS[i+1]));

            const auto scale=std::max(0.5*(incomingScale+outgoingScale),1e-12);
            const auto sourceLength=std::max(3.0*(incomingScale+outgoingScale),1e-12);
            const auto targetDerivative=scaled(subtract(end.curvature,start.curvature),
                1.0/sourceLength);
            auto maximumCurvatureError=0.0;
            auto maximumDerivativeError=0.0;
            for(unsigned sample=0;sample<12;++sample) {
                const auto parameter=3.0*(static_cast<double>(sample)+0.5)/12.0;
                const auto first=evaluate(firstControls,FIRST_KNOTS,2,parameter);
                const auto second=evaluate(secondControls,SECOND_KNOTS,1,parameter);
                const auto segment=std::min<std::size_t>(2,
                    static_cast<std::size_t>(std::floor(parameter)));
                const auto third=thirdControls[segment];
                const auto speed=length(first);
                if(speed<=1e-12) return std::numeric_limits<double>::infinity();
                const auto tangent=scaled(first,1.0/speed);
                const auto curvature=scaled(subtract(second,
                    scaled(tangent,dot(second,tangent))),1.0/(speed*speed));
                const auto target=add(scaled(start.curvature,1.0-parameter/3.0),
                    scaled(end.curvature,parameter/3.0));
                maximumCurvatureError=std::max(maximumCurvatureError,
                    length(subtract(curvature,target)));

                const auto firstSecond=dot(first,second);
                const auto inverseSpeed2=1.0/(speed*speed);
                const auto inverseSpeed4=inverseSpeed2*inverseSpeed2;
                const auto inverseSpeed6=inverseSpeed4*inverseSpeed2;
                const auto derivative=scaled(add(scaled(third,inverseSpeed2),
                    add(scaled(second,-3.0*firstSecond*inverseSpeed4),
                        add(scaled(first,-(dot(second,second)+dot(first,third))*inverseSpeed4),
                            scaled(first,4.0*firstSecond*firstSecond*inverseSpeed6)))),1.0/speed);
                maximumDerivativeError=std::max(maximumDerivativeError,
                    length(subtract(derivative,targetDerivative)));
            }
            const auto deviation=std::pow(incoming/initialIncoming-1.0,2)
                +std::pow(outgoing/initialOutgoing-1.0,2);
            return maximumCurvatureError*scale
                +0.05*maximumDerivativeError*scale*scale+0.01*deviation;
        };

        auto bestIncoming=initialIncoming;
        auto bestOutgoing=initialOutgoing;
        auto bestScore=score(bestIncoming,bestOutgoing);
        const auto consider=[&](const double incoming,const double outgoing) {
            const auto boundedIncoming=std::clamp(incoming,0.25*incomingScale,2.0*incomingScale);
            const auto boundedOutgoing=std::clamp(outgoing,0.25*outgoingScale,2.0*outgoingScale);
            const auto candidateScore=score(boundedIncoming,boundedOutgoing);
            if(candidateScore<bestScore) {
                bestScore=candidateScore;
                bestIncoming=boundedIncoming;
                bestOutgoing=boundedOutgoing;
            }
        };
        const auto startCurved=length(start.curvature)>1e-12;
        const auto endCurved=length(end.curvature)>1e-12;
        if(startCurved&&endCurved) {
            for(const auto offset:{-0.15,0.0,0.15})
                consider(initialIncoming*(1.0+offset),initialOutgoing*(1.0+offset));
        } else if(startCurved||endCurved) {
            for(const auto offset:{-0.15,0.15}) {
                consider(initialIncoming*(1.0+offset),initialOutgoing);
                consider(initialIncoming,initialOutgoing*(1.0+offset));
            }
        }
        return {bestIncoming,bestOutgoing};
    }
}
