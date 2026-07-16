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

    struct MicroEntityCluster {
        std::size_t left=0;
        std::size_t firstCollapsed=0;
        std::size_t lastCollapsed=0;
        std::size_t right=0;
    };

    struct ShortEntitySplineCluster {
        std::size_t left=0;
        std::size_t firstInterior=0;
        std::size_t lastInterior=0;
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
                result.push_back({left,left+1,right-1,right});
                left=right;
            } else ++left;
        }
        return result;
    }

    inline std::vector<double> evenlySpacedCompositeControlDistances(
            const std::span<const double> entityLengths,const double programmedScale) {
        std::vector<double> result;
        if(entityLengths.empty()||!std::isfinite(programmedScale)||programmedScale<=0.0)
            return result;
        auto totalLength=0.0;
        for(const auto length:entityLengths) {
            if(!std::isfinite(length)||length<=0.0) return {};
            totalLength+=length;
        }
        if(!std::isfinite(totalLength)||totalLength<=0.0) return {};
        const auto desired=std::max<std::size_t>(1,static_cast<std::size_t>(
            std::llround(totalLength/programmedScale)));
        const auto controlCount=std::min(entityLengths.size(),desired);
        result.reserve(controlCount);
        for(std::size_t control=0;control<controlCount;++control)
            result.push_back(totalLength*(static_cast<double>(control)+0.5)
                /static_cast<double>(controlCount));
        return result;
    }

    struct ClusterEntity3 {
        double length=0.0;
        std::array<Vector3,3> positions{};
        std::array<Vector3,3> tangents{};
    };

    inline std::vector<MicroEntityCluster> detectMicroEntityClusters(
            const std::span<const double> lengths,const double programmedScale) {
        std::vector<MicroEntityCluster> result;
        if(!std::isfinite(programmedScale)||programmedScale<=0.0||lengths.size()<3)
            return result;
        const auto longThreshold=6.0*programmedScale;
        for(std::size_t left=0;left+2<lengths.size();) {
            if(lengths[left]<=longThreshold) {
                ++left;
                continue;
            }
            auto total=0.0;
            auto right=left+1;
            while(right<lengths.size()&&lengths[right]<=longThreshold
                  &&total+lengths[right]<=programmedScale*(1.0+1e-12)) {
                total+=lengths[right];
                ++right;
            }
            if(right>left+1&&right<lengths.size()&&lengths[right]>longThreshold) {
                result.push_back({left,left+1,right-1,right});
                left=right;
            } else ++left;
        }
        return result;
    }

    inline bool bridgeApproximatesEntityChain(const std::span<const ClusterEntity3> entities,
            const std::size_t left,const std::size_t right,const double programmedScale) {
        if(right<=left+1) return false;
        auto middleLength=0.0;
        for(auto entity=left+1;entity<right;++entity) middleLength+=entities[entity].length;
        if(!std::isfinite(middleLength)||middleLength<=programmedScale
           ||middleLength>6.0*programmedScale*(1.0+1e-12)||right-left-1>16)
            return false;
        const auto start=entities[left].positions[2];
        const auto end=entities[right].positions[0];
        const auto displacement=subtract(end,start);
        const auto directLength=length(displacement);
        if(directLength<=1e-12||middleLength>1.5*directLength) return false;

        auto previousTangent=entities[left].tangents[2];
        const auto checkTangent=[&](Vector3 tangent) {
            const auto previousLength=length(previousTangent);
            const auto tangentLength=length(tangent);
            if(previousLength<=1e-12||tangentLength<=1e-12) return false;
            previousTangent=scaled(previousTangent,1.0/previousLength);
            tangent=scaled(tangent,1.0/tangentLength);
            if(dot(previousTangent,tangent)<0.7071067811865476) return false;
            previousTangent=tangent;
            return true;
        };
        for(auto entity=left+1;entity<right;++entity)
            for(const auto tangent:entities[entity].tangents)
                if(!checkTangent(tangent)) return false;
        if(!checkTangent(entities[right].tangents[0])) return false;

        const auto handle=middleLength/3.0;
        const std::array<Vector3,4> controls {
            start,
            add(start,scaled(entities[left].tangents[2],handle)),
            subtract(end,scaled(entities[right].tangents[0],handle)),
            end,
        };
        const auto evaluateBezier=[&](const double u) {
            const auto oneMinus=1.0-u;
            return add(add(scaled(controls[0],oneMinus*oneMinus*oneMinus),
                           scaled(controls[1],3.0*oneMinus*oneMinus*u)),
                       add(scaled(controls[2],3.0*oneMinus*u*u),
                           scaled(controls[3],u*u*u)));
        };
        auto distance=0.0;
        auto maximumDeviation=0.0;
        for(auto entity=left+1;entity<right;++entity) {
            const auto &source=entities[entity];
            for(std::size_t sample=0;sample<2;++sample) {
                const auto localFraction=sample==0?0.5:1.0;
                const auto sourceDistance=distance+localFraction*source.length;
                const auto candidate=evaluateBezier(std::clamp(
                    sourceDistance/middleLength,0.0,1.0));
                maximumDeviation=std::max(maximumDeviation,
                    length(subtract(source.positions[sample+1],candidate)));
            }
            distance+=source.length;
        }
        return maximumDeviation<=0.25*programmedScale*(1.0+1e-12);
    }

    inline std::vector<MicroEntityCluster> detectMicroEntityClusters(
            const std::span<const ClusterEntity3> entities,const double programmedScale) {
        std::vector<MicroEntityCluster> result;
        if(!std::isfinite(programmedScale)||programmedScale<=0.0||entities.size()<3)
            return result;
        const auto longThreshold=6.0*programmedScale;
        for(std::size_t left=0;left+2<entities.size();) {
            if(entities[left].length<=longThreshold) {
                ++left;
                continue;
            }
            auto total=0.0;
            auto right=left+1;
            while(right<entities.size()&&entities[right].length<=longThreshold
                  &&right-left-1<16
                  &&total+entities[right].length<=6.0*programmedScale*(1.0+1e-12)) {
                total+=entities[right].length;
                ++right;
            }
            if(right>left+1&&right<entities.size()&&entities[right].length>longThreshold
               &&(total<=programmedScale*(1.0+1e-12)
                  ||bridgeApproximatesEntityChain(entities,left,right,programmedScale))) {
                result.push_back({left,left+1,right-1,right});
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

    // Fair only the unconstrained controls of a clamped cubic B-spline. The first
    // and last three controls establish the existing endpoint position, tangent,
    // and curvature construction and therefore remain immutable. This is control-
    // polygon conditioning, not interpolation or fit-point spline construction.
    //
    // The objective combines fidelity to the original controls with the squared
    // third finite differences of the control polygon. For a uniform interior
    // cubic span that finite difference is proportional to its constant third
    // parameter derivative. A candidate is retained only if a separate analytic
    // B-spline measurement confirms that it reduces peak arc-length curvature
    // derivative; parameter-space fairness alone is not accepted as evidence.
    template<std::size_t Dimension>
    inline std::vector<std::array<double,Dimension>> conditionCubicSplineInteriorControls(
            const std::span<const std::array<double,Dimension>> original,
            const double programmedScale) {
        std::vector<std::array<double,Dimension>> result(original.begin(),original.end());
        if(result.size()<=6||!std::isfinite(programmedScale)||programmedScale<=0.0)
            return result;

        constexpr std::array<double,4> THIRD_DIFFERENCE{-1.0,3.0,-3.0,1.0};
        constexpr double SMOOTHING_WEIGHT=0.15;
        constexpr double MAXIMUM_CONTROL_DISPLACEMENT_IN_P=0.20;
        constexpr unsigned RELAXATION_SWEEPS=12;
        const auto maximumDisplacement=
            MAXIMUM_CONTROL_DISPLACEMENT_IN_P*programmedScale;
        const auto firstFree=std::size_t {3};
        const auto lastFree=result.size()-4;

        const auto relaxControl=[&](const std::size_t control) {
            auto diagonal=1.0;
            auto rightHandSide=original[control];
            const auto firstRow=control>3?control-3:0;
            const auto lastRow=std::min(control,result.size()-4);
            for(auto row=firstRow;row<=lastRow;++row) {
                const auto local=control-row;
                const auto coefficient=THIRD_DIFFERENCE[local];
                diagonal+=SMOOTHING_WEIGHT*coefficient*coefficient;
                for(std::size_t other=0;other<THIRD_DIFFERENCE.size();++other) {
                    if(other==local) continue;
                    const auto weighted=-SMOOTHING_WEIGHT*coefficient
                        *THIRD_DIFFERENCE[other];
                    for(std::size_t axis=0;axis<Dimension;++axis)
                        rightHandSide[axis]+=weighted*result[row+other][axis];
                }
            }
            for(std::size_t axis=0;axis<Dimension;++axis) {
                const auto candidate=rightHandSide[axis]/diagonal;
                result[control][axis]=std::clamp(candidate,
                    original[control][axis]-maximumDisplacement,
                    original[control][axis]+maximumDisplacement);
            }
        };

        for(unsigned sweep=0;sweep<RELAXATION_SWEEPS;++sweep) {
            for(auto control=firstFree;control<=lastFree;++control)
                relaxControl(control);
            for(auto control=lastFree+1;control-->firstFree;)
                relaxControl(control);
        }

        const auto maximumCurvatureDerivative=[](
                const std::span<const std::array<double,Dimension>> controls) {
            const auto zero=[] { return std::array<double,Dimension>{}; };
            const auto scaledArray=[&](const std::array<double,Dimension> &value,
                                       const double amount) {
                auto output=zero();
                for(std::size_t axis=0;axis<Dimension;++axis)
                    output[axis]=value[axis]*amount;
                return output;
            };
            const auto subtractArray=[&](const std::array<double,Dimension> &left,
                                         const std::array<double,Dimension> &right) {
                auto output=zero();
                for(std::size_t axis=0;axis<Dimension;++axis)
                    output[axis]=left[axis]-right[axis];
                return output;
            };
            const auto dotArray=[](const std::array<double,Dimension> &left,
                                   const std::array<double,Dimension> &right) {
                auto output=0.0;
                for(std::size_t axis=0;axis<Dimension;++axis)
                    output+=left[axis]*right[axis];
                return output;
            };
            const auto parameterMaximum=static_cast<double>(controls.size()-3);
            std::vector<double> knots(controls.size()+4,parameterMaximum);
            std::fill_n(knots.begin(),4,0.0);
            for(std::size_t index=4;index<controls.size();++index)
                knots[index]=static_cast<double>(index-3);
            std::vector<double> firstKnots(knots.begin()+1,knots.end()-1);
            std::vector<double> secondKnots(firstKnots.begin()+1,firstKnots.end()-1);
            std::vector<double> thirdKnots(secondKnots.begin()+1,secondKnots.end()-1);
            std::vector<std::array<double,Dimension>> first(controls.size()-1);
            std::vector<std::array<double,Dimension>> second(controls.size()-2);
            std::vector<std::array<double,Dimension>> third(controls.size()-3);
            for(std::size_t index=0;index<first.size();++index)
                first[index]=scaledArray(subtractArray(controls[index+1],controls[index]),
                    3.0/(knots[index+4]-knots[index+1]));
            for(std::size_t index=0;index<second.size();++index)
                second[index]=scaledArray(subtractArray(first[index+1],first[index]),
                    2.0/(firstKnots[index+3]-firstKnots[index+1]));
            for(std::size_t index=0;index<third.size();++index)
                third[index]=scaledArray(subtractArray(second[index+1],second[index]),
                    1.0/(secondKnots[index+2]-secondKnots[index+1]));

            const auto evaluateDynamic=[&](const auto &values,const auto &valueKnots,
                                           const std::size_t degree,
                                           const double requestedParameter) {
                if(requestedParameter<=valueKnots[degree]) return values.front();
                const auto endParameter=valueKnots[values.size()];
                if(requestedParameter>=endParameter) return values.back();
                const auto parameter=std::clamp(
                    requestedParameter,valueKnots[degree],endParameter);
                auto span=degree;
                while(span+1<values.size()&&parameter>=valueKnots[span+1]) ++span;
                std::array<std::array<double,Dimension>,4> work{};
                for(std::size_t index=0;index<=degree;++index)
                    work[index]=values[span-degree+index];
                for(std::size_t level=1;level<=degree;++level)
                    for(std::size_t index=degree;index>=level;--index) {
                        const auto knotIndex=span-degree+index;
                        const auto denominator=valueKnots[knotIndex+degree-level+1]
                            -valueKnots[knotIndex];
                        const auto alpha=(parameter-valueKnots[knotIndex])/denominator;
                        for(std::size_t axis=0;axis<Dimension;++axis)
                            work[index][axis]=std::lerp(
                                work[index-1][axis],work[index][axis],alpha);
                    }
                return work[degree];
            };

            auto maximum=0.0;
            constexpr unsigned INTERIOR_SAMPLES_PER_SPAN=16;
            for(std::size_t span=0;span+3<controls.size();++span) {
                for(unsigned sample=0;sample<=INTERIOR_SAMPLES_PER_SPAN+1;++sample) {
                    double parameter=0.0;
                    if(sample==0) parameter=static_cast<double>(span);
                    else if(sample==INTERIOR_SAMPLES_PER_SPAN+1)
                        parameter=std::nextafter(static_cast<double>(span+1),
                                                 static_cast<double>(span));
                    else parameter=static_cast<double>(span)
                        +(static_cast<double>(sample)-0.5)/INTERIOR_SAMPLES_PER_SPAN;
                    const auto r1=evaluateDynamic(first,firstKnots,2,parameter);
                    const auto r2=evaluateDynamic(second,secondKnots,1,parameter);
                    const auto r3=evaluateDynamic(third,thirdKnots,0,parameter);
                    const auto speedSquared=dotArray(r1,r1);
                    if(!std::isfinite(speedSquared)||speedSquared<=1e-24)
                        return std::numeric_limits<double>::infinity();
                    const auto speed=std::sqrt(speedSquared);
                    const auto firstSecond=dotArray(r1,r2);
                    const auto inverseSpeed2=1.0/speedSquared;
                    const auto inverseSpeed4=inverseSpeed2*inverseSpeed2;
                    const auto inverseSpeed6=inverseSpeed4*inverseSpeed2;
                    auto parameterDerivative=zero();
                    for(std::size_t axis=0;axis<Dimension;++axis)
                        parameterDerivative[axis]=r3[axis]*inverseSpeed2
                            -3.0*r2[axis]*firstSecond*inverseSpeed4
                            -r1[axis]*(dotArray(r2,r2)+dotArray(r1,r3))*inverseSpeed4
                            +4.0*r1[axis]*firstSecond*firstSecond*inverseSpeed6;
                    const auto derivative=scaledArray(parameterDerivative,1.0/speed);
                    const auto magnitudeSquared=dotArray(derivative,derivative);
                    if(!std::isfinite(magnitudeSquared))
                        return std::numeric_limits<double>::infinity();
                    maximum=std::max(maximum,std::sqrt(magnitudeSquared));
                }
            }
            return maximum;
        };
        const auto originalMaximum=maximumCurvatureDerivative(original);
        const auto candidateMaximum=maximumCurvatureDerivative(result);
        if(!std::isfinite(candidateMaximum)
           ||candidateMaximum>=originalMaximum*(1.0-1e-9))
            return std::vector<std::array<double,Dimension>>(original.begin(),original.end());
        return result;
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
