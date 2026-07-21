#include "machine/SplineReconstruction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <numeric>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

namespace ngc {
    namespace {
        constexpr std::array AXIS_COMPONENTS {
            &position_t::x, &position_t::y, &position_t::z,
            &position_t::a, &position_t::b, &position_t::c,
        };

        position_t scaled(const position_t &value, const double amount) {
            return { value.x*amount, value.y*amount, value.z*amount,
                     value.a*amount, value.b*amount, value.c*amount };
        }
        position_t add(const position_t &a, const position_t &b) { return a+b; }
        position_t subtract(const position_t &a, const position_t &b) { return a-b; }
        double positionDot(const position_t &left,const position_t &right) {
            return left.x*right.x+left.y*right.y+left.z*right.z
                +left.a*right.a+left.b*right.b+left.c*right.c;
        }

        position_t evaluateBSpline(const std::span<const position_t> controls,
                                   const std::span<const double> knots,
                                   const std::size_t degree, const double requestedParameter) {
            const auto endParameter=knots[controls.size()];
            if(requestedParameter<=knots[degree]) return controls.front();
            if(requestedParameter>=endParameter) return controls.back();
            const auto parameter=std::clamp(requestedParameter,knots[degree],endParameter);
            auto span=degree;
            while(span+1<controls.size()&&parameter>=knots[span+1]) ++span;
            std::array<position_t,6> work{};
            for(std::size_t index=0;index<=degree;++index)
                work[index]=controls[span-degree+index];
            for(std::size_t level=1;level<=degree;++level) {
                for(std::size_t index=degree;index>=level;--index) {
                    const auto knotIndex=span-degree+index;
                    const auto denominator=knots[knotIndex+degree-level+1]-knots[knotIndex];
                    const auto alpha=(parameter-knots[knotIndex])/denominator;
                    work[index]=add(scaled(work[index-1],1.0-alpha),scaled(work[index],alpha));
                }
            }
            return work[degree];
        }

        using spline_detail::SplineFitSolver;

        struct FittingSpline {
            std::size_t degree=0;
            std::vector<position_t> controls;
            std::vector<double> knots;

            position_t at(const double parameter) const {
                return evaluateBSpline(controls,knots,degree,parameter);
            }

            FittingSpline derivative() const {
                FittingSpline result;
                result.degree=degree-1;
                result.knots={knots.begin()+1,knots.end()-1};
                result.controls.resize(controls.size()-1);
                for(std::size_t index=0;index<result.controls.size();++index) {
                    const auto denominator=knots[index+degree+1]-knots[index+1];
                    result.controls[index]=scaled(subtract(controls[index+1],controls[index]),
                        static_cast<double>(degree)/denominator);
                }
                return result;
            }
        };

        std::vector<double> openSplineKnots(const std::size_t controlCount,
                                            const std::size_t degree) {
            const auto spans=controlCount-degree;
            std::vector<double> knots(controlCount+degree+1,static_cast<double>(spans));
            std::fill_n(knots.begin(),degree+1,0.0);
            for(std::size_t index=degree+1;index<controlCount;++index)
                knots[index]=static_cast<double>(index-degree);
            return knots;
        }

        std::vector<double> endpointDerivativeCoefficients(
                const FittingSpline &spline,const std::size_t order,const bool end) {
            const auto count=spline.controls.size();
            std::vector<std::vector<double>> values(count,std::vector<double>(count));
            for(std::size_t index=0;index<count;++index) values[index][index]=1.0;
            auto knots=spline.knots;
            auto degree=spline.degree;
            for(std::size_t derivative=0;derivative<order;++derivative) {
                std::vector<std::vector<double>> next(
                    values.size()-1,std::vector<double>(count));
                for(std::size_t index=0;index<next.size();++index) {
                    const auto factor=static_cast<double>(degree)
                        /(knots[index+degree+1]-knots[index+1]);
                    for(std::size_t source=0;source<count;++source)
                        next[index][source]=factor
                            *(values[index+1][source]-values[index][source]);
                }
                values=std::move(next);
                knots={knots.begin()+1,knots.end()-1};
                --degree;
            }
            return end?values.back():values.front();
        }

        void imposeEndpointDerivative(FittingSpline &spline,const std::size_t order,
                                      const position_t &desired,const bool end) {
            const auto coefficients=endpointDerivativeCoefficients(spline,order,end);
            const auto unknown=end?spline.controls.size()-1-order:order;
            const auto &origin=end?spline.controls.back():spline.controls.front();
            const auto coefficientSum=std::accumulate(
                coefficients.begin(),coefficients.end(),0.0);
            for(const auto component:AXIS_COMPONENTS) {
                auto residual=desired.*component-origin.*component*coefficientSum;
                for(std::size_t index=0;index<spline.controls.size();++index)
                    if(index!=unknown)
                        residual-=coefficients[index]
                            *(spline.controls[index].*component-origin.*component);
                spline.controls[unknown].*component=origin.*component
                    +residual/coefficients[unknown];
            }
        }

        using SplineFitSource=spline_detail::SplineReconstructionSource;

        FittingSpline initialQuinticSpline(const std::size_t spans,
                                           const SplineFitSource &source) {
            FittingSpline result;
            result.degree=5;
            result.controls.resize(spans+result.degree);
            result.knots=openSplineKnots(result.controls.size(),result.degree);
            for(std::size_t index=0;index<result.controls.size();++index) {
                auto greville=0.0;
                for(std::size_t offset=1;offset<=result.degree;++offset)
                    greville+=result.knots[index+offset];
                greville/=static_cast<double>(result.degree);
                result.controls[index]=source.positionAt(
                    source.length*greville/static_cast<double>(spans));
            }
            const auto parameterDistance=source.length/static_cast<double>(spans);
            result.controls.front()=source.positionAt(0.0);
            result.controls.back()=source.positionAt(source.length);
            imposeEndpointDerivative(result,1,
                scaled(source.startTangent,parameterDistance),false);
            imposeEndpointDerivative(result,2,
                scaled(source.startCurvature,parameterDistance*parameterDistance),false);
            imposeEndpointDerivative(result,3,scaled(source.startCurvatureDerivative,
                parameterDistance*parameterDistance*parameterDistance),false);
            imposeEndpointDerivative(result,1,
                scaled(source.endTangent,parameterDistance),true);
            imposeEndpointDerivative(result,2,
                scaled(source.endCurvature,parameterDistance*parameterDistance),true);
            imposeEndpointDerivative(result,3,scaled(source.endCurvatureDerivative,
                parameterDistance*parameterDistance*parameterDistance),true);
            return result;
        }

        struct SplineFitMeasurement {
            double maximumNormalCurvatureDerivative=0.0;
            double integratedNormalCurvatureDerivative=0.0;
            double maximumCurvature=0.0;
            double maximumDeviation=0.0;
            double minimumPermissibleVelocity=std::numeric_limits<double>::infinity();
            double estimatedVelocityLimitedDuration=0.0;
        };

        std::vector<double> knotIntervalFeeds(const SplineFitSource &source,
                                               const std::size_t spans) {
            std::vector<double> result(spans,std::numeric_limits<double>::infinity());
            if(source.boundaries.size()!=source.programmedFeeds.size()+1
               ||source.boundaries.empty()) return result;
            auto sourceIndex=std::size_t{0};
            for(std::size_t span=0;span<spans;++span) {
                const auto sourceFrom=source.length*static_cast<double>(span)/spans;
                const auto sourceTo=source.length*static_cast<double>(span+1)/spans;
                while(sourceIndex+1<source.programmedFeeds.size()
                      &&source.boundaries[sourceIndex+1]<=sourceFrom)
                    ++sourceIndex;
                for(auto candidate=sourceIndex;
                    candidate<source.programmedFeeds.size()
                        &&source.boundaries[candidate]<sourceTo;
                    ++candidate) {
                    const auto overlapFrom=std::max(
                        sourceFrom,source.boundaries[candidate]);
                    const auto overlapTo=std::min(
                        sourceTo,source.boundaries[candidate+1]);
                    if(overlapTo>overlapFrom)
                        result[span]=std::min(
                            result[span],source.programmedFeeds[candidate]);
                }
            }
            return result;
        }

        double permissibleVelocity(const SplineFitSource &source,
                const double programmedFeed,
                const position_t &tangent,const position_t &curvature,
                const position_t &curvatureDerivative) {
            auto result=programmedFeed;
            const auto reduce=[&](const double candidate) {
                if(std::isfinite(candidate)&&candidate>0.0) result=std::min(result,candidate);
            };
            const auto &limits=source.velocityLimits;
            for(const auto component:AXIS_COMPONENTS) {
                const auto tangentComponent=std::abs(tangent.*component);
                const auto axisVelocity=limits.axisVelocity.*component;
                if(tangentComponent>1e-15&&std::isfinite(axisVelocity))
                    reduce(axisVelocity/tangentComponent);
                const auto curvatureComponent=std::abs(curvature.*component);
                const auto axisAcceleration=limits.axisAcceleration.*component;
                if(curvatureComponent>1e-15&&std::isfinite(axisAcceleration))
                    reduce(std::sqrt(axisAcceleration/curvatureComponent));
                const auto derivativeComponent=std::abs(curvatureDerivative.*component);
                const auto axisJerk=limits.axisJerk.*component;
                if(derivativeComponent>1e-15&&std::isfinite(axisJerk))
                    reduce(std::cbrt(axisJerk/derivativeComponent));
            }
            const auto curvatureMagnitude=curvature.length();
            if(curvatureMagnitude>1e-15&&std::isfinite(limits.pathAcceleration))
                reduce(std::sqrt(limits.pathAcceleration/curvatureMagnitude));
            const auto derivativeMagnitude=curvatureDerivative.length();
            if(derivativeMagnitude>1e-15&&std::isfinite(limits.pathJerk))
                reduce(std::cbrt(limits.pathJerk/derivativeMagnitude));
            return result;
        }

        SplineFitMeasurement measureSplineFit(const FittingSpline &spline,
                                              const SplineFitSource &source,
                                              const unsigned samplesPerSpan,
                                              const bool measureVelocity=false) {
            const auto first=spline.derivative();
            const auto second=first.derivative();
            const auto third=second.derivative();
            const auto spans=spline.controls.size()-spline.degree;
            const auto intervalFeeds=measureVelocity
                ?knotIntervalFeeds(source,spans):std::vector<double>{};
            SplineFitMeasurement result;
            auto previous=spline.at(0.0);
            auto previousNormal=0.0;
            auto previousVelocity=std::numeric_limits<double>::infinity();
            for(std::size_t sample=0;sample<=spans*samplesPerSpan;++sample) {
                const auto parameter=static_cast<double>(sample)/samplesPerSpan;
                const auto position=spline.at(parameter);
                const auto r1=first.at(parameter);
                const auto r2=second.at(parameter);
                const auto r3=third.at(parameter);
                const auto speed=r1.length();
                if(!std::isfinite(speed)||speed<=1e-15) {
                    result.maximumCurvature=std::numeric_limits<double>::infinity();
                    result.maximumNormalCurvatureDerivative=
                        std::numeric_limits<double>::infinity();
                    result.maximumDeviation=std::numeric_limits<double>::infinity();
                    return result;
                }
                const auto tangent=scaled(r1,1.0/speed);
                const auto firstSecond=positionDot(r1,r2);
                const auto inverseSpeed2=1.0/(speed*speed);
                const auto inverseSpeed4=inverseSpeed2*inverseSpeed2;
                const auto inverseSpeed6=inverseSpeed4*inverseSpeed2;
                const auto curvature=scaled(subtract(r2,
                    scaled(tangent,positionDot(r2,tangent))),inverseSpeed2);
                const auto parameterDerivative=add(scaled(r3,inverseSpeed2),
                    add(scaled(r2,-3.0*firstSecond*inverseSpeed4),
                        add(scaled(r1,-(positionDot(r2,r2)+positionDot(r1,r3))
                            *inverseSpeed4),
                            scaled(r1,4.0*firstSecond*firstSecond*inverseSpeed6))));
                const auto derivative=scaled(parameterDerivative,1.0/speed);
                const auto normal=subtract(derivative,
                    scaled(tangent,positionDot(tangent,derivative))).length();
                auto permissible=std::numeric_limits<double>::infinity();
                if(measureVelocity) {
                    const auto interval=std::min(spans-1,
                        static_cast<std::size_t>(std::floor(parameter)));
                    auto programmedFeed=intervalFeeds[interval];
                    if(interval>0&&parameter==static_cast<double>(interval))
                        programmedFeed=std::min(
                            programmedFeed,intervalFeeds[interval-1]);
                    permissible=permissibleVelocity(source,programmedFeed,
                        tangent,curvature,derivative);
                    result.minimumPermissibleVelocity=std::min(
                        result.minimumPermissibleVelocity,permissible);
                }
                result.maximumNormalCurvatureDerivative=std::max(
                    result.maximumNormalCurvatureDerivative,normal);
                result.maximumCurvature=std::max(result.maximumCurvature,curvature.length());
                result.maximumDeviation=std::max(result.maximumDeviation,
                    subtract(position,source.positionAt(source.length*parameter/spans)).length());
                if(sample!=0) {
                    const auto distance=subtract(position,previous).length();
                    result.integratedNormalCurvatureDerivative+=
                        0.5*(previousNormal+normal)*distance;
                    if(measureVelocity) {
                        const auto intervalVelocity=std::min(previousVelocity,permissible);
                        if(!std::isinf(intervalVelocity)) {
                            if(std::isfinite(intervalVelocity)&&intervalVelocity>0.0)
                                result.estimatedVelocityLimitedDuration+=distance/intervalVelocity;
                            else result.estimatedVelocityLimitedDuration=
                                std::numeric_limits<double>::infinity();
                        }
                    }
                }
                previous=position;
                previousNormal=normal;
                previousVelocity=permissible;
            }
            return result;
        }

        std::expected<void,std::string> certifySplineFitTube(
                const FittingSpline &spline,const SplineFitSource &source,
                const double deviationLimit) {
            if(source.boundaries.size()<2||source.boundaries.front()!=0.0
               ||source.boundaries.back()!=source.length)
                return std::unexpected("quintic source has invalid ordered boundaries");
            const auto second=spline.derivative().derivative();
            auto maximumSecond=0.0;
            for(const auto &control:second.controls)
                maximumSecond=std::max(maximumSecond,control.length());
            const auto spans=static_cast<double>(spline.controls.size()-spline.degree);
            const auto parameterAtSourceDistance=[&](const double distance) {
                return spans*distance/source.length;
            };
            struct Interval { double from=0.0; double to=0.0; unsigned depth=0; };
            std::vector<Interval> pending;
            pending.reserve(source.boundaries.size()*2);
            for(std::size_t index=0;index+1<source.boundaries.size();++index)
                pending.push_back({source.boundaries[index],source.boundaries[index+1],0});
            const auto attemptBudget=std::max<std::size_t>(8192,4*spline.controls.size());
            auto attempts=std::size_t {0};
            while(!pending.empty()) {
                if(++attempts>attemptBudget)
                    return std::unexpected(std::format(
                        "quintic source-tube proof exceeded {} ordered intervals",attemptBudget));
                const auto interval=pending.back();
                pending.pop_back();
                const auto fromParameter=parameterAtSourceDistance(interval.from);
                const auto toParameter=parameterAtSourceDistance(interval.to);
                const auto fromDeviation=subtract(spline.at(fromParameter),
                    source.positionAt(interval.from)).length();
                const auto toDeviation=subtract(spline.at(toParameter),
                    source.positionAt(interval.to)).length();
                const auto sourceError=source.chordErrorBound(interval.from,interval.to);
                const auto parameterWidth=toParameter-fromParameter;
                const auto splineError=maximumSecond*parameterWidth*parameterWidth/8.0;
                const auto bound=std::max(fromDeviation,toDeviation)+sourceError+splineError;
                if(std::isfinite(bound)&&bound<=deviationLimit) continue;
                if(interval.depth>=20)
                    return std::unexpected(std::format(
                        "quintic source-tube proof did not converge: bound={} limit={} "
                        "source=[{},{}]",bound,deviationLimit,interval.from,interval.to));
                const auto middle=std::midpoint(interval.from,interval.to);
                pending.push_back({middle,interval.to,interval.depth+1});
                pending.push_back({interval.from,middle,interval.depth+1});
            }
            return {};
        }

        struct SplineFitResult {
            FittingSpline spline;
            std::size_t candidateCount=0;
        };

        std::expected<SplineFitResult,std::string> fitQuinticSpline(
                FittingSpline initial,const SplineFitSource &source,
                const double programmedScale,const SplineFitSolver solver) {
            constexpr std::size_t FIXED_CONTROLS_PER_END=4;
            constexpr std::size_t HALF_BANDWIDTH=3;
            constexpr std::array<double,4> THIRD_DIFFERENCE{-1.0,3.0,-3.0,1.0};
            const auto firstFree=FIXED_CONTROLS_PER_END;
            const auto lastFree=initial.controls.size()-FIXED_CONTROLS_PER_END;
            if(lastFree<=firstFree) return SplineFitResult{std::move(initial),0};
            const auto limit=programmedScale;
            const auto velocityTargeted=
                solver==SplineFitSolver::VelocityTargetedBandedFairness;
            const auto original=measureSplineFit(initial,source,32,velocityTargeted);
            if(original.maximumDeviation>limit*(1.0+1e-9))
                return std::unexpected("initial quintic exceeds the programmed G64 scale");
            const auto normalSharpnessScore=[&](const SplineFitMeasurement &measurement) {
                const auto peak=measurement.maximumNormalCurvatureDerivative
                    /std::max(original.maximumNormalCurvatureDerivative,1e-30);
                const auto integral=measurement.integratedNormalCurvatureDerivative
                    /std::max(original.integratedNormalCurvatureDerivative,1e-30);
                return 0.65*peak+0.35*integral;
            };
            const auto score=[&](const SplineFitMeasurement &measurement) {
                if(!velocityTargeted) return normalSharpnessScore(measurement);
                if(!std::isfinite(measurement.estimatedVelocityLimitedDuration)
                   ||measurement.estimatedVelocityLimitedDuration<=0.0)
                    return std::numeric_limits<double>::infinity();
                const auto duration=measurement.estimatedVelocityLimitedDuration
                    /std::max(original.estimatedVelocityLimitedDuration,1e-30);
                auto bottleneck=1.0;
                if(std::isfinite(original.minimumPermissibleVelocity)
                   &&original.minimumPermissibleVelocity>0.0
                   &&std::isfinite(measurement.minimumPermissibleVelocity)
                   &&measurement.minimumPermissibleVelocity>0.0)
                    bottleneck=original.minimumPermissibleVelocity
                        /measurement.minimumPermissibleVelocity;
                return 0.8*duration+0.2*bottleneck;
            };
            const auto acceptable=[&](const SplineFitMeasurement &measurement) {
                return measurement.maximumDeviation<=0.98*limit
                    &&measurement.maximumCurvature<=original.maximumCurvature*1.02;
            };
            SplineFitResult result{initial,0};
            auto bestScore=score(original);

            if(solver==SplineFitSolver::CoordinateSearch) {
                const auto coordinateScore=[&](const SplineFitMeasurement &measurement) {
                    const auto excess=std::max(
                        0.0,measurement.maximumDeviation/(0.98*limit)-1.0);
                    return score(measurement)+100.0*excess*excess;
                };
                bestScore=coordinateScore(original);
                for(const auto fraction:{0.25,0.0625,0.015625}) {
                    const auto step=programmedScale*fraction;
                    for(std::size_t control=firstFree;control<lastFree;++control) {
                        for(const auto component:AXIS_COMPONENTS) {
                            for(const auto direction:{-1.0,1.0}) {
                                auto candidate=result.spline;
                                candidate.controls[control].*component+=direction*step;
                                const auto measurement=measureSplineFit(candidate,source,4);
                                ++result.candidateCount;
                                if(measurement.maximumCurvature
                                        >original.maximumCurvature*1.02) continue;
                                const auto candidateScore=coordinateScore(measurement);
                                if(candidateScore<bestScore*(1.0-1e-10)) {
                                    result.spline=std::move(candidate);
                                    bestScore=candidateScore;
                                }
                            }
                        }
                    }
                }
                const auto verified=measureSplineFit(result.spline,source,256);
                if(verified.maximumDeviation>limit*(1.0+1e-9))
                    return std::unexpected(
                        "coordinate-search quintic exceeds the programmed G64 scale");
                return result;
            }

            const auto freeCount=lastFree-firstFree;
            const auto rowCount=initial.controls.size()-3;
            const std::vector<double> uniformRowWeights(rowCount,1.0);
            const auto solve=[&](const double fairnessWeight,
                                 const std::span<const double> rowWeights)
                    ->std::optional<FittingSpline> {
                std::vector<std::array<double,2*HALF_BANDWIDTH+1>> matrix(freeCount);
                std::vector<position_t> rightHandSide(freeCount);
                for(std::size_t free=0;free<freeCount;++free) {
                    matrix[free][HALF_BANDWIDTH]=1.0;
                    rightHandSide[free]=initial.controls[firstFree+free];
                }
                for(std::size_t row=0;row+3<initial.controls.size();++row) {
                    for(std::size_t a=0;a<THIRD_DIFFERENCE.size();++a) {
                        const auto controlA=row+a;
                        if(controlA<firstFree||controlA>=lastFree) continue;
                        const auto freeA=controlA-firstFree;
                        for(std::size_t b=0;b<THIRD_DIFFERENCE.size();++b) {
                            const auto controlB=row+b;
                            const auto value=fairnessWeight*rowWeights[row]
                                *THIRD_DIFFERENCE[a]*THIRD_DIFFERENCE[b];
                            if(controlB>=firstFree&&controlB<lastFree) {
                                const auto freeB=controlB-firstFree;
                                if(freeB<=freeA)
                                    matrix[freeA][HALF_BANDWIDTH+freeB-freeA]+=value;
                            } else {
                                for(const auto component:AXIS_COMPONENTS)
                                    rightHandSide[freeA].*component-=
                                        value*(initial.controls[controlB].*component);
                            }
                        }
                    }
                }
                std::vector<std::array<double,HALF_BANDWIDTH+1>> lower(freeCount);
                for(std::size_t i=0;i<freeCount;++i) {
                    const auto firstJ=i>HALF_BANDWIDTH?i-HALF_BANDWIDTH:0;
                    for(auto j=firstJ;j<=i;++j) {
                        auto value=matrix[i][HALF_BANDWIDTH+j-i];
                        const auto firstK=std::max(
                            i>HALF_BANDWIDTH?i-HALF_BANDWIDTH:0,
                            j>HALF_BANDWIDTH?j-HALF_BANDWIDTH:0);
                        for(auto k=firstK;k<j;++k)
                            value-=lower[i][i-k]*lower[j][j-k];
                        if(i==j) {
                            if(!std::isfinite(value)||value<=1e-18) return std::nullopt;
                            lower[i][0]=std::sqrt(value);
                        } else lower[i][i-j]=value/lower[j][0];
                    }
                }
                auto candidate=initial;
                for(const auto component:AXIS_COMPONENTS) {
                    std::vector<double> intermediate(freeCount),solution(freeCount);
                    for(std::size_t i=0;i<freeCount;++i) {
                        auto value=rightHandSide[i].*component;
                        const auto maximum=std::min(HALF_BANDWIDTH,i);
                        for(std::size_t distance=1;distance<=maximum;++distance)
                            value-=lower[i][distance]*intermediate[i-distance];
                        intermediate[i]=value/lower[i][0];
                    }
                    for(auto reverse=freeCount;reverse-->0;) {
                        auto value=intermediate[reverse];
                        const auto maximum=std::min(
                            HALF_BANDWIDTH,freeCount-1-reverse);
                        for(std::size_t distance=1;distance<=maximum;++distance)
                            value-=lower[reverse+distance][distance]
                                *solution[reverse+distance];
                        solution[reverse]=value/lower[reverse][0];
                        candidate.controls[firstFree+reverse].*component=solution[reverse];
                    }
                }
                return candidate;
            };

            constexpr std::array<double,13> FAIRNESS_WEIGHTS{
                3e-2,1e-1,3e-1,1.0,3.0,10.0,30.0,100.0,300.0,500.0,700.0,
                1000.0,3000.0};
            struct ActiveSeed {
                FittingSpline spline;
                double fairnessWeight=0.0;
                double score=std::numeric_limits<double>::infinity();
            };
            std::array<std::optional<ActiveSeed>,3> activeSeeds;
            for(const auto fairnessWeight:FAIRNESS_WEIGHTS) {
                auto candidate=solve(fairnessWeight,uniformRowWeights);
                ++result.candidateCount;
                if(!candidate) continue;
                const auto measurement=measureSplineFit(
                    *candidate,source,8,velocityTargeted);
                if(!acceptable(measurement)) continue;
                const auto candidateScore=score(measurement);
                const auto utilization=measurement.maximumDeviation/limit;
                const auto seedIndex=utilization<0.35?0U:utilization<0.7?1U:2U;
                if(!activeSeeds[seedIndex]||candidateScore<activeSeeds[seedIndex]->score)
                    activeSeeds[seedIndex]=ActiveSeed{
                        *candidate,fairnessWeight,candidateScore};
                if(candidateScore<bestScore*(1.0-1e-10)) {
                    result.spline=std::move(*candidate);
                    bestScore=candidateScore;
                }
            }
            if(solver==SplineFitSolver::PeakTargetedBandedFairness) {
                constexpr unsigned ACTIVE_PASSES=4;
                constexpr unsigned PROFILE_SAMPLES_PER_SPAN=24;
                for(const auto &seed:activeSeeds) {
                    if(!seed) continue;
                    auto current=seed->spline;
                    auto currentScore=score(measureSplineFit(current,source,12));
                    auto rowWeights=uniformRowWeights;
                    const auto spans=current.controls.size()-current.degree;
                    for(unsigned pass=0;pass<ACTIVE_PASSES;++pass) {
                        const auto first=current.derivative();
                        const auto second=first.derivative();
                        const auto third=second.derivative();
                        std::vector<double> spanPeaks(spans);
                        auto globalPeak=0.0;
                        for(std::size_t sample=0;
                                sample<=spans*PROFILE_SAMPLES_PER_SPAN;++sample) {
                            const auto parameter=static_cast<double>(sample)
                                /PROFILE_SAMPLES_PER_SPAN;
                            const auto r1=first.at(parameter);
                            const auto r2=second.at(parameter);
                            const auto r3=third.at(parameter);
                            const auto speed=r1.length();
                            if(!std::isfinite(speed)||speed<=1e-15) {
                                globalPeak=std::numeric_limits<double>::infinity();
                                break;
                            }
                            const auto tangent=scaled(r1,1.0/speed);
                            const auto firstSecond=positionDot(r1,r2);
                            const auto inverseSpeed2=1.0/(speed*speed);
                            const auto inverseSpeed4=inverseSpeed2*inverseSpeed2;
                            const auto inverseSpeed6=inverseSpeed4*inverseSpeed2;
                            const auto parameterDerivative=add(scaled(r3,inverseSpeed2),
                                add(scaled(r2,-3.0*firstSecond*inverseSpeed4),
                                    add(scaled(r1,-(positionDot(r2,r2)+positionDot(r1,r3))
                                        *inverseSpeed4),scaled(r1,4.0*firstSecond*firstSecond
                                        *inverseSpeed6))));
                            const auto derivative=scaled(parameterDerivative,1.0/speed);
                            const auto normal=subtract(derivative,
                                scaled(tangent,positionDot(tangent,derivative))).length();
                            const auto span=std::min(spans-1,
                                static_cast<std::size_t>(std::floor(parameter)));
                            spanPeaks[span]=std::max(spanPeaks[span],normal);
                            globalPeak=std::max(globalPeak,normal);
                        }
                        if(!std::isfinite(globalPeak)||globalPeak<=1e-30) break;
                        auto changed=false;
                        for(std::size_t span=0;span<spans;++span) {
                            const auto ratio=spanPeaks[span]/globalPeak;
                            if(ratio<0.35) continue;
                            const auto increase=1.0+12.0*ratio*ratio*ratio*ratio;
                            for(std::size_t local=0;local<3;++local) {
                                const auto row=std::min(rowCount-1,span+local);
                                const auto updated=std::min(1e6,rowWeights[row]*increase);
                                changed=changed||updated>rowWeights[row]*(1.0+1e-12);
                                rowWeights[row]=updated;
                            }
                        }
                        if(!changed) break;
                        auto candidate=solve(seed->fairnessWeight,rowWeights);
                        ++result.candidateCount;
                        if(!candidate) break;
                        const auto measurement=measureSplineFit(*candidate,source,16);
                        if(!acceptable(measurement)) break;
                        const auto candidateScore=score(measurement);
                        if(candidateScore>=currentScore*(1.0-1e-10)) break;
                        current=std::move(*candidate);
                        currentScore=candidateScore;
                        if(candidateScore<bestScore*(1.0-1e-10)) {
                            result.spline=current;
                            bestScore=candidateScore;
                        }
                    }
                }
            }
            const auto verified=measureSplineFit(result.spline,source,256,velocityTargeted);
            if(verified.maximumDeviation>limit*(1.0+1e-9))
                return std::unexpected("banded quintic exceeds the programmed G64 scale");
            return result;
        }

    }

    std::expected<spline_detail::ReconstructedSpline,std::string>
    spline_detail::reconstructSpline(const std::span<const position_t> cubicControls,
            const SplineReconstructionSource &source,const double programmedScale,
            const bool certifyTube,const SplineFitSolver solver) {
        if(cubicControls.size()<=6
           ||solver==SplineFitSolver::None)
            return ReconstructedSpline{
                .degree=3,
                .controls={cubicControls.begin(),cubicControls.end()},
            };
        auto fitted=fitQuinticSpline(
            initialQuinticSpline(cubicControls.size()-3,source),source,programmedScale,
            solver);
        if(!fitted) return std::unexpected(fitted.error());
        if(certifyTube) {
            if(auto certified=certifySplineFitTube(
                    fitted->spline,source,programmedScale);!certified)
                return std::unexpected(certified.error());
        }
        return ReconstructedSpline{
            .degree=fitted->spline.degree,
            .controls=std::move(fitted->spline.controls),
        };
    }

}
