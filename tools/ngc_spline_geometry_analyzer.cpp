#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
    using Point=std::array<double,6>;

    Point subtract(const Point &a,const Point &b) {
        Point result{};
        for(std::size_t axis=0;axis<result.size();++axis) result[axis]=a[axis]-b[axis];
        return result;
    }
    Point added(const Point &a,const Point &b) {
        Point result{};
        for(std::size_t axis=0;axis<result.size();++axis) result[axis]=a[axis]+b[axis];
        return result;
    }
    Point scaled(const Point &value,const double scale) {
        Point result{};
        for(std::size_t axis=0;axis<result.size();++axis) result[axis]=value[axis]*scale;
        return result;
    }
    double dot(const Point &a,const Point &b) {
        return std::inner_product(a.begin(),a.end(),b.begin(),0.0);
    }
    double length(const Point &value) { return std::sqrt(dot(value,value)); }

    Point evaluate(const std::span<const Point> controls,const std::span<const double> knots,
                   const std::size_t degree,const double requested) {
        if(requested<=knots[degree]) return controls.front();
        const auto maximum=knots[controls.size()];
        if(requested>=maximum) return controls.back();
        const auto parameter=std::clamp(requested,knots[degree],maximum);
        auto span=degree;
        while(span+1<controls.size()&&parameter>=knots[span+1]) ++span;
        std::array<Point,4> work{};
        for(std::size_t index=0;index<=degree;++index)
            work[index]=controls[span-degree+index];
        for(std::size_t level=1;level<=degree;++level)
            for(std::size_t index=degree;index>=level;--index) {
                const auto knot=span-degree+index;
                const auto denominator=knots[knot+degree-level+1]-knots[knot];
                const auto alpha=(parameter-knots[knot])/denominator;
                for(std::size_t axis=0;axis<6;++axis)
                    work[index][axis]=std::lerp(work[index-1][axis],work[index][axis],alpha);
            }
        return work[degree];
    }

    struct Spline {
        std::size_t id=0;
        std::size_t horizon=0;
        std::size_t firstSource=0;
        std::size_t lastSource=0;
        std::vector<Point> controls;
        std::vector<double> boundaries;
    };

    struct DifferentialSample {
        Point position{};
        Point tangent{};
        Point curvature{};
        Point curvatureDerivative{};
    };

    class SplineEvaluator {
        const Spline &spline;
        std::vector<double> knots,firstKnots,secondKnots,thirdKnots;
        std::vector<Point> first,second,third;
    public:
        explicit SplineEvaluator(const Spline &value):spline(value) {
            const auto count=spline.controls.size();
            const auto maximumParameter=static_cast<double>(count-3);
            knots.assign(count+4,maximumParameter);
            std::fill_n(knots.begin(),4,0.0);
            for(std::size_t index=4;index<count;++index) knots[index]=index-3;
            firstKnots=std::vector<double>(knots.begin()+1,knots.end()-1);
            secondKnots=std::vector<double>(firstKnots.begin()+1,firstKnots.end()-1);
            thirdKnots=std::vector<double>(secondKnots.begin()+1,secondKnots.end()-1);
            first.resize(count-1);
            second.resize(count-2);
            third.resize(count-3);
            for(std::size_t index=0;index<first.size();++index)
                first[index]=scaled(subtract(spline.controls[index+1],spline.controls[index]),
                    3.0/(knots[index+4]-knots[index+1]));
            for(std::size_t index=0;index<second.size();++index)
                second[index]=scaled(subtract(first[index+1],first[index]),
                    2.0/(firstKnots[index+3]-firstKnots[index+1]));
            for(std::size_t index=0;index<third.size();++index)
                third[index]=scaled(subtract(second[index+1],second[index]),
                    1.0/(secondKnots[index+2]-secondKnots[index+1]));
        }

        DifferentialSample at(const double parameter) const {
            DifferentialSample result;
            result.position=evaluate(spline.controls,knots,3,parameter);
            const auto r1=evaluate(first,firstKnots,2,parameter);
            const auto r2=evaluate(second,secondKnots,1,parameter);
            const auto r3=evaluate(third,thirdKnots,0,parameter);
            const auto speed=length(r1);
            if(speed<=1e-15) throw std::runtime_error("singular spline parameter speed");
            result.tangent=scaled(r1,1.0/speed);
            const auto h=dot(r1,r2);
            const auto inverse2=1.0/(speed*speed);
            const auto inverse4=inverse2*inverse2;
            const auto inverse6=inverse4*inverse2;
            for(std::size_t axis=0;axis<6;++axis) {
                result.curvature[axis]=r2[axis]*inverse2-r1[axis]*h*inverse4;
                const auto derivativeU=r3[axis]*inverse2-3.0*r2[axis]*h*inverse4
                    -r1[axis]*(dot(r2,r2)+dot(r1,r3))*inverse4
                    +4.0*r1[axis]*h*h*inverse6;
                result.curvatureDerivative[axis]=derivativeU/speed;
            }
            return result;
        }
    };

    struct FeedProfilePoint {
        double distance=0.0;
        Point acceleration{};
        Point jerk{};
        double accelerationMagnitude=0.0;
        double jerkMagnitude=0.0;
        double tangentialJerkMagnitude=0.0;
        double normalJerkMagnitude=0.0;
    };

    void writeFeedProfile(const Spline &spline,const double feedPerMinute,
                          const std::filesystem::path &basePath) {
        if(!std::isfinite(feedPerMinute)||feedPerMinute<=0.0)
            throw std::runtime_error("profile feed must be positive and finite");
        const auto velocity=feedPerMinute/60.0;
        const SplineEvaluator evaluator(spline);
        constexpr unsigned SAMPLES_PER_SPAN=128;
        const auto sampleCount=(spline.controls.size()-3)*SAMPLES_PER_SPAN;
        std::vector<FeedProfilePoint> profile;
        profile.reserve(sampleCount+1);
        Point previousPosition{};
        for(std::size_t index=0;index<=sampleCount;++index) {
            const auto parameter=static_cast<double>(spline.controls.size()-3)*index/sampleCount;
            const auto sample=evaluator.at(parameter);
            FeedProfilePoint point;
            if(index>0) point.distance=profile.back().distance
                +length(subtract(sample.position,previousPosition));
            point.acceleration=scaled(sample.curvature,velocity*velocity);
            point.jerk=scaled(sample.curvatureDerivative,velocity*velocity*velocity);
            point.accelerationMagnitude=length(point.acceleration);
            point.jerkMagnitude=length(point.jerk);
            const auto tangentialJerk=dot(sample.tangent,point.jerk);
            point.tangentialJerkMagnitude=std::abs(tangentialJerk);
            point.normalJerkMagnitude=length(subtract(
                point.jerk,scaled(sample.tangent,tangentialJerk)));
            profile.push_back(point);
            previousPosition=sample.position;
        }

        auto csvPath=basePath;
        csvPath.replace_extension("csv");
        std::ofstream csv(csvPath,std::ios::trunc);
        csv.precision(17);
        csv<<"distance,time,acceleration_x,acceleration_y,acceleration_z,"
            "acceleration_magnitude,jerk_x,jerk_y,jerk_z,jerk_magnitude,"
            "turning_jerk_magnitude,curvature_change_jerk_magnitude\n";
        for(const auto &point:profile)
            csv<<point.distance<<','<<point.distance/velocity<<','
                <<point.acceleration[0]<<','<<point.acceleration[1]<<','
                <<point.acceleration[2]<<','<<point.accelerationMagnitude<<','
                <<point.jerk[0]<<','<<point.jerk[1]<<','<<point.jerk[2]<<','
                <<point.jerkMagnitude<<','<<point.tangentialJerkMagnitude<<','
                <<point.normalJerkMagnitude<<'\n';
        csv.flush();
        if(!csv) throw std::runtime_error("incomplete profile CSV write");

        auto svgPath=basePath;
        svgPath.replace_extension("svg");
        std::ofstream svg(svgPath,std::ios::trunc);
        constexpr double WIDTH=1200.0,LEFT=90.0,RIGHT=25.0;
        constexpr double PANEL_HEIGHT=260.0;
        const auto plotWidth=WIDTH-LEFT-RIGHT;
        const auto totalDistance=profile.back().distance;
        const auto maximumAcceleration=std::max(20.0,
            std::ranges::max(profile,{},&FeedProfilePoint::accelerationMagnitude)
                .accelerationMagnitude)*1.05;
        const auto maximumJerk=std::max(100.0,
            std::ranges::max(profile,{},&FeedProfilePoint::jerkMagnitude).jerkMagnitude)*1.05;
        svg<<"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1200\" height=\"760\" "
            "viewBox=\"0 0 1200 760\"><rect width=\"1200\" height=\"760\" fill=\"#fff\"/>"
            "<style>text{font-family:Segoe UI,Arial,sans-serif;fill:#222}.grid{stroke:#ddd;stroke-width:1}"
            ".axis{stroke:#222;stroke-width:1.5}.limit{stroke:#d62728;stroke-width:2;stroke-dasharray:8 5}"
            ".mag{fill:none;stroke:#111;stroke-width:2}.x{fill:none;stroke:#1677ff;stroke-width:1.4}"
            ".y{fill:none;stroke:#18a558;stroke-width:1.4}.tangent{fill:none;stroke:#e67e22;stroke-width:1.5}"
            ".normal{fill:none;stroke:#8e44ad;stroke-width:1.5}</style>";
        svg<<"<text x=\"90\" y=\"30\" font-size=\"21\" font-weight=\"600\">Spline "
            <<spline.id<<" (sources "<<spline.firstSource<<"–"<<spline.lastSource
            <<") at constant F"<<feedPerMinute<<" = "<<velocity<<" in/s</text>";
        const auto drawPanel=[&](const double top,const double maximum,const double limit,
                                 const bool jerkPanel,const std::string_view title,
                                 const std::string_view units) {
            const auto xAt=[&](const double distance) { return LEFT+plotWidth*distance/totalDistance; };
            const auto yAt=[&](const double value) { return top+PANEL_HEIGHT*(1.0-value/maximum); };
            for(unsigned grid=0;grid<=4;++grid) {
                const auto value=maximum*grid/4.0;
                const auto y=yAt(value);
                svg<<"<line class=\"grid\" x1=\""<<LEFT<<"\" y1=\""<<y
                    <<"\" x2=\""<<WIDTH-RIGHT<<"\" y2=\""<<y<<"\"/>"
                    <<"<text x=\""<<LEFT-8<<"\" y=\""<<y+4
                    <<"\" font-size=\"12\" text-anchor=\"end\">"<<value<<"</text>";
                const auto distance=totalDistance*grid/4.0;
                const auto x=xAt(distance);
                svg<<"<line class=\"grid\" x1=\""<<x<<"\" y1=\""<<top
                    <<"\" x2=\""<<x<<"\" y2=\""<<top+PANEL_HEIGHT<<"\"/>"
                    <<"<text x=\""<<x<<"\" y=\""<<top+PANEL_HEIGHT+18
                    <<"\" font-size=\"12\" text-anchor=\"middle\">"<<distance<<"</text>";
            }
            svg<<"<line class=\"axis\" x1=\""<<LEFT<<"\" y1=\""<<top
                <<"\" x2=\""<<LEFT<<"\" y2=\""<<top+PANEL_HEIGHT<<"\"/>"
                <<"<line class=\"axis\" x1=\""<<LEFT<<"\" y1=\""<<top+PANEL_HEIGHT
                <<"\" x2=\""<<WIDTH-RIGHT<<"\" y2=\""<<top+PANEL_HEIGHT<<"\"/>"
                <<"<text x=\""<<LEFT<<"\" y=\""<<top-10<<"\" font-size=\"16\" font-weight=\"600\">"
                <<title<<" ("<<units<<")</text>";
            if(limit<=maximum) svg<<"<line class=\"limit\" x1=\""<<LEFT<<"\" y1=\""<<yAt(limit)
                <<"\" x2=\""<<WIDTH-RIGHT<<"\" y2=\""<<yAt(limit)<<"\"/>";
            const auto polyline=[&](const std::string_view css,const auto valueAt) {
                svg<<"<polyline class=\""<<css<<"\" points=\"";
                for(const auto &point:profile) svg<<xAt(point.distance)<<','<<yAt(valueAt(point))<<' ';
                svg<<"\"/>";
            };
            polyline("mag",[&](const FeedProfilePoint &point) {
                return jerkPanel?point.jerkMagnitude:point.accelerationMagnitude;
            });
            if(jerkPanel) {
                polyline("tangent",[](const FeedProfilePoint &point) {
                    return point.tangentialJerkMagnitude;
                });
                polyline("normal",[](const FeedProfilePoint &point) {
                    return point.normalJerkMagnitude;
                });
            } else {
                polyline("x",[](const FeedProfilePoint &point) {
                    return std::abs(point.acceleration[0]);
                });
                polyline("y",[](const FeedProfilePoint &point) {
                    return std::abs(point.acceleration[1]);
                });
            }
        };
        drawPanel(70.0,maximumAcceleration,20.0,false,"Acceleration","in/s²");
        drawPanel(410.0,maximumJerk,100.0,true,"Jerk","in/s³");
        svg<<"<text x=\"600\" y=\"720\" font-size=\"15\" text-anchor=\"middle\">Distance along spline (in)</text>"
            "<line x1=\"90\" y1=\"742\" x2=\"125\" y2=\"742\" class=\"mag\"/><text x=\"132\" y=\"747\" font-size=\"13\">vector magnitude</text>"
            "<line x1=\"300\" y1=\"742\" x2=\"335\" y2=\"742\" class=\"x\"/><text x=\"342\" y=\"747\" font-size=\"13\">|X|</text>"
            "<line x1=\"410\" y1=\"742\" x2=\"445\" y2=\"742\" class=\"y\"/><text x=\"452\" y=\"747\" font-size=\"13\">|Y|</text>"
            "<line x1=\"520\" y1=\"742\" x2=\"555\" y2=\"742\" class=\"tangent\"/><text x=\"562\" y=\"747\" font-size=\"13\">turning jerk</text>"
            "<line x1=\"690\" y1=\"742\" x2=\"725\" y2=\"742\" class=\"normal\"/><text x=\"732\" y=\"747\" font-size=\"13\">curvature-change jerk</text>"
            "<line x1=\"910\" y1=\"742\" x2=\"945\" y2=\"742\" class=\"limit\"/><text x=\"952\" y=\"747\" font-size=\"13\">path limit</text></svg>";
        svg.flush();
        if(!svg) throw std::runtime_error("incomplete profile SVG write");
        std::cout<<"wrote constant-feed profile to "<<csvPath.string()<<" and "
            <<svgPath.string()<<'\n';
    }

    struct Measurement {
        Spline spline;
        double length=0.0;
        double minimumControlSpacing=0.0;
        double maximumControlSpacing=0.0;
        double spacingRatio=0.0;
        double minimumParameterSpeed=0.0;
        double maximumCurvature=0.0;
        double maximumCurvatureDerivative=0.0;
        double maximumCurvatureDerivativeParameter=0.0;
        double curvatureAtMaximumDerivative=0.0;
        double tangentialDerivativeAtMaximum=0.0;
        double normalDerivativeAtMaximum=0.0;
        double maximumNormalCurvatureDerivative=0.0;
        double integratedNormalCurvatureDerivative=0.0;
        double derivativeTraversalCost=0.0;
        double derivativeVelocityCap=0.0;
    };

    Measurement measure(Spline spline) {
        const auto count=spline.controls.size();
        const auto maximumParameter=static_cast<double>(count-3);
        std::vector<double> knots(count+4,maximumParameter);
        std::fill_n(knots.begin(),4,0.0);
        for(std::size_t index=4;index<count;++index) knots[index]=index-3;
        std::vector<double> firstKnots(knots.begin()+1,knots.end()-1);
        std::vector<double> secondKnots(firstKnots.begin()+1,firstKnots.end()-1);
        std::vector<double> thirdKnots(secondKnots.begin()+1,secondKnots.end()-1);
        std::vector<Point> first(count-1),second(count-2),third(count-3);
        for(std::size_t index=0;index<first.size();++index)
            first[index]=scaled(subtract(spline.controls[index+1],spline.controls[index]),
                3.0/(knots[index+4]-knots[index+1]));
        for(std::size_t index=0;index<second.size();++index)
            second[index]=scaled(subtract(first[index+1],first[index]),
                2.0/(firstKnots[index+3]-firstKnots[index+1]));
        for(std::size_t index=0;index<third.size();++index)
            third[index]=scaled(subtract(second[index+1],second[index]),
                1.0/(secondKnots[index+2]-secondKnots[index+1]));

        Measurement result{.spline=std::move(spline)};
        result.length=result.spline.boundaries.empty()?0.0:result.spline.boundaries.back();
        result.minimumControlSpacing=std::numeric_limits<double>::infinity();
        for(std::size_t index=1;index<count;++index) {
            const auto spacing=length(subtract(result.spline.controls[index],
                                               result.spline.controls[index-1]));
            result.minimumControlSpacing=std::min(result.minimumControlSpacing,spacing);
            result.maximumControlSpacing=std::max(result.maximumControlSpacing,spacing);
        }
        result.spacingRatio=result.minimumControlSpacing>0.0
            ?result.maximumControlSpacing/result.minimumControlSpacing
            :std::numeric_limits<double>::infinity();
        result.minimumParameterSpeed=std::numeric_limits<double>::infinity();
        std::vector<double> spanMaximumDerivative(count-3,0.0);
        std::vector<double> spanLengths(count-3,0.0);
        constexpr unsigned SAMPLES_PER_SPAN=32;
        for(std::size_t span=0;span+3<count;++span) {
            double previousSpeed=0.0;
            double previousNormalDerivative=0.0;
            for(unsigned sample=0;sample<=SAMPLES_PER_SPAN;++sample) {
                auto parameter=static_cast<double>(span)
                    +static_cast<double>(sample)/SAMPLES_PER_SPAN;
                if(sample==SAMPLES_PER_SPAN&&span+4<count)
                    parameter=std::nextafter(parameter,static_cast<double>(span));
                const auto r1=evaluate(first,firstKnots,2,parameter);
                const auto r2=evaluate(second,secondKnots,1,parameter);
                const auto r3=evaluate(third,thirdKnots,0,parameter);
                const auto speed=length(r1);
                const auto distanceStep=sample>0
                    ?(previousSpeed+speed)*0.5/SAMPLES_PER_SPAN:0.0;
                if(sample>0) spanLengths[span]+=distanceStep;
                result.minimumParameterSpeed=std::min(result.minimumParameterSpeed,speed);
                if(speed<=1e-15) {
                    result.maximumCurvature=std::numeric_limits<double>::infinity();
                    result.maximumCurvatureDerivative=std::numeric_limits<double>::infinity();
                    continue;
                }
                const auto h=dot(r1,r2);
                const auto inverse2=1.0/(speed*speed);
                const auto inverse4=inverse2*inverse2;
                const auto inverse6=inverse4*inverse2;
                Point curvature{};
                Point derivative{};
                for(std::size_t axis=0;axis<6;++axis) {
                    curvature[axis]=r2[axis]*inverse2-r1[axis]*h*inverse4;
                    const auto derivativeU=r3[axis]*inverse2-3.0*r2[axis]*h*inverse4
                        -r1[axis]*(dot(r2,r2)+dot(r1,r3))*inverse4
                        +4.0*r1[axis]*h*h*inverse6;
                    derivative[axis]=derivativeU/speed;
                }
                result.maximumCurvature=std::max(result.maximumCurvature,length(curvature));
                const auto derivativeMagnitude=length(derivative);
                const auto tangent=scaled(r1,1.0/speed);
                const auto tangential=dot(tangent,derivative);
                const auto normalDerivative=length(
                    subtract(derivative,scaled(tangent,tangential)));
                result.maximumNormalCurvatureDerivative=std::max(
                    result.maximumNormalCurvatureDerivative,normalDerivative);
                if(sample>0) result.integratedNormalCurvatureDerivative+=
                    0.5*(previousNormalDerivative+normalDerivative)*distanceStep;
                previousSpeed=speed;
                previousNormalDerivative=normalDerivative;
                spanMaximumDerivative[span]=std::max(
                    spanMaximumDerivative[span],derivativeMagnitude);
                if(derivativeMagnitude>result.maximumCurvatureDerivative) {
                    result.maximumCurvatureDerivative=derivativeMagnitude;
                    result.maximumCurvatureDerivativeParameter=parameter;
                    result.curvatureAtMaximumDerivative=length(curvature);
                    result.tangentialDerivativeAtMaximum=std::abs(tangential);
                    result.normalDerivativeAtMaximum=normalDerivative;
                }
            }
        }
        for(std::size_t firstSpan=0;firstSpan<spanMaximumDerivative.size();firstSpan+=3) {
            auto groupMaximum=0.0;
            auto groupLength=0.0;
            for(auto span=firstSpan;
                span<std::min(firstSpan+3,spanMaximumDerivative.size());++span) {
                groupMaximum=std::max(groupMaximum,spanMaximumDerivative[span]);
                groupLength+=spanLengths[span];
            }
            result.derivativeTraversalCost+=groupLength*std::cbrt(groupMaximum/100.0);
        }
        result.derivativeVelocityCap=result.maximumCurvatureDerivative>0.0
            ?std::cbrt(100.0/result.maximumCurvatureDerivative)
            :std::numeric_limits<double>::infinity();
        return result;
    }

    Measurement relaxInteriorControls(const Measurement &original,
            const double programmedScale,const double maximumDisplacementInP) {
        auto best=original;
        if(best.spline.controls.size()<=6||best.length<0.35||best.length>0.8
           ||best.derivativeVelocityCap>=0.35) return best;
        const auto originalControls=original.spline.controls;
        for(const auto fraction:{0.25,0.125,0.0625,0.03125}) {
            const auto step=programmedScale*fraction;
            auto improved=true;
            for(unsigned sweep=0;sweep<2&&improved;++sweep) {
                improved=false;
                for(std::size_t control=3;control+3<best.spline.controls.size();++control)
                    for(std::size_t axis=0;axis<2;++axis)
                        for(const auto direction:{-1.0,1.0}) {
                            auto candidateSpline=best.spline;
                            candidateSpline.controls[control][axis]=std::clamp(
                                candidateSpline.controls[control][axis]+direction*step,
                                originalControls[control][axis]
                                    -maximumDisplacementInP*programmedScale,
                                originalControls[control][axis]
                                    +maximumDisplacementInP*programmedScale);
                            auto candidate=measure(std::move(candidateSpline));
                            if(candidate.derivativeTraversalCost
                                    <best.derivativeTraversalCost*(1.0-1e-12)
                               &&candidate.maximumCurvature<=original.maximumCurvature*1.05
                               &&candidate.maximumCurvatureDerivative
                                    <=original.maximumCurvatureDerivative*1.10) {
                                best=std::move(candidate);
                                improved=true;
                            }
                        }
            }
        }
        return best;
    }

    double maximumCurveDeviation(const Spline &reference,const Spline &candidate) {
        if(reference.controls.size()!=candidate.controls.size())
            return std::numeric_limits<double>::infinity();
        const SplineEvaluator referenceEvaluator(reference);
        const SplineEvaluator candidateEvaluator(candidate);
        constexpr unsigned SAMPLES_PER_SPAN=48;
        const auto samples=(reference.controls.size()-3)*SAMPLES_PER_SPAN;
        auto maximum=0.0;
        for(std::size_t sample=0;sample<=samples;++sample) {
            const auto parameter=static_cast<double>(reference.controls.size()-3)
                *sample/samples;
            maximum=std::max(maximum,length(subtract(
                referenceEvaluator.at(parameter).position,
                candidateEvaluator.at(parameter).position)));
        }
        return maximum;
    }

    struct PhysicalJerkOptimization {
        Measurement measurement;
        double maximumCurveDeviation=0.0;
    };

    PhysicalJerkOptimization optimizePhysicalJerk(const Measurement &original,
            const double programmedScale,const double maximumCurveDeviationInP) {
        PhysicalJerkOptimization best{original,0.0};
        if(original.spline.controls.size()<=6||!std::isfinite(maximumCurveDeviationInP)
           ||maximumCurveDeviationInP<=0.0
           ||original.maximumNormalCurvatureDerivative<=0.0
           ||original.integratedNormalCurvatureDerivative<=0.0) return best;
        const auto deviationLimit=programmedScale*maximumCurveDeviationInP;
        const auto score=[&](const Measurement &measurement) {
            const auto peak=measurement.maximumNormalCurvatureDerivative
                /original.maximumNormalCurvatureDerivative;
            const auto integral=measurement.integratedNormalCurvatureDerivative
                /original.integratedNormalCurvatureDerivative;
            return 0.65*peak+0.35*integral
                +4.0*std::max(0.0,peak-1.0)
                +2.0*std::max(0.0,integral-1.0);
        };
        auto bestScore=score(best.measurement);
        const SplineEvaluator originalEvaluator(original.spline);
        const auto startGeometry=originalEvaluator.at(0.0);
        const auto endGeometry=originalEvaluator.at(
            static_cast<double>(original.spline.controls.size()-3));
        const auto consider=[&](Spline candidateSpline) {
            const auto deviation=maximumCurveDeviation(original.spline,candidateSpline);
            if(deviation>deviationLimit*(1.0+1e-12)) return false;
            auto candidate=measure(std::move(candidateSpline));
            if(candidate.maximumCurvature>original.maximumCurvature*1.02) return false;
            const auto candidateScore=score(candidate);
            if(candidateScore>=bestScore*(1.0-1e-10)) return false;
            best={std::move(candidate),deviation};
            bestScore=candidateScore;
            return true;
        };
        for(const auto fraction:{0.5,0.25,0.125,0.0625,0.03125,0.015625}) {
            const auto step=programmedScale*fraction;
            auto improved=true;
            for(unsigned sweep=0;sweep<3&&improved;++sweep) {
                improved=false;
                for(unsigned variable=0;variable<4;++variable) {
                    for(const auto direction:{-1.0,1.0}) {
                        auto candidate=best.measurement.spline;
                        const auto last=candidate.controls.size()-1;
                        if(variable<2) {
                            const auto h=dot(subtract(candidate.controls[1],
                                candidate.controls[0]),startGeometry.tangent);
                            const auto tangentDistance=dot(subtract(candidate.controls[2],
                                candidate.controls[0]),startGeometry.tangent);
                            const auto nextH=variable==0?h+direction*step:h;
                            const auto nextTangentDistance=variable==1
                                ?tangentDistance+direction*step:tangentDistance;
                            if(nextH<=1e-8) continue;
                            candidate.controls[1]=added(candidate.controls[0],
                                scaled(startGeometry.tangent,nextH));
                            candidate.controls[2]=added(added(candidate.controls[0],
                                scaled(startGeometry.tangent,nextTangentDistance)),
                                scaled(startGeometry.curvature,3.0*nextH*nextH));
                        } else {
                            const auto h=dot(subtract(candidate.controls[last],
                                candidate.controls[last-1]),endGeometry.tangent);
                            const auto tangentDistance=dot(subtract(candidate.controls[last-2],
                                candidate.controls[last]),endGeometry.tangent);
                            const auto nextH=variable==2?h+direction*step:h;
                            const auto nextTangentDistance=variable==3
                                ?tangentDistance+direction*step:tangentDistance;
                            if(nextH<=1e-8) continue;
                            candidate.controls[last-1]=subtract(candidate.controls[last],
                                scaled(endGeometry.tangent,nextH));
                            candidate.controls[last-2]=added(added(candidate.controls[last],
                                scaled(endGeometry.tangent,nextTangentDistance)),
                                scaled(endGeometry.curvature,3.0*nextH*nextH));
                        }
                        if(consider(std::move(candidate))) improved=true;
                    }
                }
                for(std::size_t control=3;
                    control+3<best.measurement.spline.controls.size();++control) {
                    for(std::size_t axis=0;axis<3;++axis) {
                        for(const auto direction:{-1.0,1.0}) {
                            auto candidateSpline=best.measurement.spline;
                            candidateSpline.controls[control][axis]+=direction*step;
                            if(consider(std::move(candidateSpline))) improved=true;
                        }
                    }
                }
            }
        }
        if(best.measurement.maximumNormalCurvatureDerivative
                >=original.maximumNormalCurvatureDerivative
           ||best.measurement.integratedNormalCurvatureDerivative
                >=original.integratedNormalCurvatureDerivative)
            return {original,0.0};
        return best;
    }

    std::vector<Spline> readSnapshot(const std::filesystem::path &path) {
        std::ifstream input(path);
        if(!input) throw std::runtime_error("could not open geometry snapshot");
        std::string word;
        input>>word;
        if(word!="ngc_spline_geometry_v1") throw std::runtime_error("unsupported snapshot");
        std::size_t primitiveCount=0;
        input>>word>>primitiveCount;
        for(std::size_t primitive=0;primitive<primitiveCount;++primitive) {
            input>>word;
            const auto values=word=="line"?14U:20U;
            double ignored=0.0;
            for(unsigned value=0;value<values;++value) input>>ignored;
        }
        std::size_t splineCount=0;
        input>>word>>splineCount;
        std::vector<Spline> result;
        result.reserve(splineCount);
        for(std::size_t index=0;index<splineCount;++index) {
            Spline spline;
            std::size_t controlCount=0,boundaryCount=0;
            input>>word>>spline.id>>spline.horizon>>spline.firstSource>>spline.lastSource
                >>controlCount>>boundaryCount;
            if(word!="spline") throw std::runtime_error("malformed spline record");
            spline.controls.resize(controlCount);
            for(auto &control:spline.controls) {
                input>>word;
                if(word!="control") throw std::runtime_error("malformed control record");
                for(auto &coordinate:control) input>>coordinate;
            }
            input>>word;
            if(word!="boundaries") throw std::runtime_error("malformed boundary record");
            spline.boundaries.resize(boundaryCount);
            for(auto &boundary:spline.boundaries) input>>boundary;
            result.push_back(std::move(spline));
        }
        if(!input) throw std::runtime_error("incomplete geometry snapshot");
        return result;
    }
}

int main(const int argc,char **argv) {
    try {
        if(argc==7&&std::string_view(argv[2])=="--profile-optimized") {
            const auto splineId=std::stoull(argv[3]);
            const auto feedPerMinute=std::stod(argv[4]);
            const auto maximumCurveDeviationInP=std::stod(argv[5]);
            const auto splines=readSnapshot(argv[1]);
            const auto found=std::ranges::find(splines,splineId,&Spline::id);
            if(found==splines.end()) throw std::runtime_error("requested spline ID not found");
            const auto original=measure(*found);
            const auto optimized=optimizePhysicalJerk(
                original,0.005,maximumCurveDeviationInP);
            writeFeedProfile(optimized.measurement.spline,feedPerMinute,argv[6]);
            const SplineEvaluator originalEvaluator(original.spline);
            const SplineEvaluator optimizedEvaluator(optimized.measurement.spline);
            const auto maximumParameter=static_cast<double>(original.spline.controls.size()-3);
            const auto originalStart=originalEvaluator.at(0.0);
            const auto optimizedStart=optimizedEvaluator.at(0.0);
            const auto originalEnd=originalEvaluator.at(maximumParameter);
            const auto optimizedEnd=optimizedEvaluator.at(maximumParameter);
            std::cout<<"optimized spline "<<splineId
                <<" normal curvature derivative peak "
                <<original.maximumNormalCurvatureDerivative<<" -> "
                <<optimized.measurement.maximumNormalCurvatureDerivative
                <<"; integral "<<original.integratedNormalCurvatureDerivative
                <<" -> "<<optimized.measurement.integratedNormalCurvatureDerivative
                <<"; maximum curve deviation "<<optimized.maximumCurveDeviation
                <<"; endpoint errors position="<<std::max(
                    length(subtract(originalStart.position,optimizedStart.position)),
                    length(subtract(originalEnd.position,optimizedEnd.position)))
                <<" tangent="<<std::max(
                    length(subtract(originalStart.tangent,optimizedStart.tangent)),
                    length(subtract(originalEnd.tangent,optimizedEnd.tangent)))
                <<" curvature="<<std::max(
                    length(subtract(originalStart.curvature,optimizedStart.curvature)),
                    length(subtract(originalEnd.curvature,optimizedEnd.curvature)))<<'\n';
            return 0;
        }
        if(argc==7&&std::string_view(argv[2])=="--profile-relaxed") {
            const auto splineId=std::stoull(argv[3]);
            const auto feedPerMinute=std::stod(argv[4]);
            const auto maximumDisplacementInP=std::stod(argv[5]);
            const auto splines=readSnapshot(argv[1]);
            const auto found=std::ranges::find(splines,splineId,&Spline::id);
            if(found==splines.end()) throw std::runtime_error("requested spline ID not found");
            const auto original=measure(*found);
            const auto relaxed=relaxInteriorControls(
                original,0.005,maximumDisplacementInP);
            writeFeedProfile(relaxed.spline,feedPerMinute,argv[6]);
            std::cout<<"relaxed spline "<<splineId
                <<" curvature derivative "<<original.maximumCurvatureDerivative
                <<" -> "<<relaxed.maximumCurvatureDerivative
                <<"; traversal cost "<<original.derivativeTraversalCost
                <<" -> "<<relaxed.derivativeTraversalCost<<'\n';
            return 0;
        }
        if(argc==6&&std::string_view(argv[2])=="--profile") {
            const auto splineId=std::stoull(argv[3]);
            const auto feedPerMinute=std::stod(argv[4]);
            const auto splines=readSnapshot(argv[1]);
            const auto found=std::ranges::find(splines,splineId,&Spline::id);
            if(found==splines.end()) throw std::runtime_error("requested spline ID not found");
            writeFeedProfile(*found,feedPerMinute,argv[5]);
            return 0;
        }
        if(argc<2||argc>4) {
            std::cerr<<"usage: ngc_spline_geometry_analyzer <snapshot.txt> "
                "[measurements.csv] [maximum-control-displacement-in-P]\n"
                "       ngc_spline_geometry_analyzer <snapshot.txt> --profile "
                "<spline-id> <feed-per-minute> <output-base>\n"
                "       ngc_spline_geometry_analyzer <snapshot.txt> --profile-relaxed "
                "<spline-id> <feed-per-minute> <maximum-control-displacement-in-P> "
                "<output-base>\n"
                "       ngc_spline_geometry_analyzer <snapshot.txt> --profile-optimized "
                "<spline-id> <feed-per-minute> <maximum-curve-deviation-in-P> "
                "<output-base>\n";
            return 2;
        }
        const auto maximumDisplacementInP=argc==4?std::stod(argv[3]):0.20;
        if(!std::isfinite(maximumDisplacementInP)||maximumDisplacementInP<=0.0)
            throw std::runtime_error("maximum displacement must be positive and finite");
        auto splines=readSnapshot(argv[1]);
        std::vector<Measurement> measurements;
        measurements.reserve(splines.size());
        for(auto &spline:splines) measurements.push_back(measure(std::move(spline)));
        std::vector<Measurement> relaxed;
        relaxed.reserve(measurements.size());
        for(const auto &measurement:measurements)
            relaxed.push_back(relaxInteriorControls(
                measurement,0.005,maximumDisplacementInP));
        const auto outputPath=argc>=3?std::filesystem::path(argv[2])
            :std::filesystem::path(argv[1]).replace_extension("csv");
        std::ofstream output(outputPath,std::ios::trunc);
        output.precision(17);
        output<<"id,horizon,first_source,last_source,controls,spans,pieces,length,min_spacing,"
            "max_spacing,spacing_ratio,min_parameter_speed,max_curvature,max_curvature_derivative,"
            "max_qs_parameter,curvature_at_max_qs,tangential_qs,normal_qs,"
            "derivative_cost,relaxed_derivative_cost,relaxed_max_curvature,"
            "relaxed_max_qs,relaxed_velocity_cap,"
            "derivative_velocity_cap\n";
        for(std::size_t index=0;index<measurements.size();++index) {
            const auto &measurement=measurements[index];
            const auto &relaxedMeasurement=relaxed[index];
            const auto &spline=measurement.spline;
            output<<spline.id<<','<<spline.horizon<<','<<spline.firstSource<<','
                <<spline.lastSource<<','<<spline.controls.size()<<','
                <<spline.controls.size()-3<<','
                <<(spline.boundaries.empty()?0:spline.boundaries.size()-1)<<','
                <<measurement.length<<','<<measurement.minimumControlSpacing<<','
                <<measurement.maximumControlSpacing<<','<<measurement.spacingRatio<<','
                <<measurement.minimumParameterSpeed<<','<<measurement.maximumCurvature<<','
                <<measurement.maximumCurvatureDerivative<<','
                <<measurement.maximumCurvatureDerivativeParameter<<','
                <<measurement.curvatureAtMaximumDerivative<<','
                <<measurement.tangentialDerivativeAtMaximum<<','
                <<measurement.normalDerivativeAtMaximum<<','
                <<measurement.derivativeTraversalCost<<','
                <<relaxedMeasurement.derivativeTraversalCost<<','
                <<relaxedMeasurement.maximumCurvature<<','
                <<relaxedMeasurement.maximumCurvatureDerivative<<','
                <<relaxedMeasurement.derivativeVelocityCap<<','
                <<measurement.derivativeVelocityCap<<'\n';
        }
        output.flush();
        if(!output) throw std::runtime_error("incomplete analyzer CSV write");
        std::cout<<"analyzed "<<measurements.size()<<" splines into "<<outputPath.string()<<'\n';
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"ERROR: "<<error.what()<<'\n';
        return 1;
    }
}
