#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "machine/ArcInterpolation.h"

namespace {
    using Point=std::array<double,6>;

    ngc::position_t position(const Point &point) {
        return {point[0],point[1],point[2],point[3],point[4],point[5]};
    }
    Point point(const ngc::position_t &position) {
        return {position.x,position.y,position.z,
                position.a,position.b,position.c};
    }

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
        std::array<Point,6> work{};
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
        std::size_t degree=3;
        std::vector<Point> controls;
        std::vector<double> boundaries;
    };

    struct Primitive {
        bool arc=false;
        std::size_t id=0;
        double feed=0.0;
        Point start{};
        Point end{};
        std::array<double,3> center{};
        std::array<double,3> axis{};
        std::shared_ptr<ngc::simulation_detail::ArcReference> arcReference;

        void prepare() {
            if(!arc) return;
            arcReference=std::make_shared<ngc::simulation_detail::ArcReference>(ngc::MoveArc(
                position(start),position(end),
                {center[0],center[1],center[2]},
                {axis[0],axis[1],axis[2]},feed));
            if(!arcReference->valid()||arcReference->length()<=1e-12)
                throw std::runtime_error("captured source contains an invalid arc");
        }

        double length() const {
            return arc?arcReference->length():lengthOfLine();
        }
        double lengthOfLine() const {
            const auto delta=subtract(end,start);
            return std::sqrt(dot(delta,delta));
        }
        Point positionAtDistance(const double requested) const {
            const auto primitiveLength=length();
            const auto distance=std::clamp(requested,0.0,primitiveLength);
            if(arc) return point(arcReference->positionAtDistance(distance));
            if(primitiveLength<=1e-15) return start;
            return added(start,scaled(subtract(end,start),distance/primitiveLength));
        }
    };

    struct Snapshot {
        std::vector<Primitive> primitives;
        std::vector<Spline> splines;
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
            const auto degree=spline.degree;
            const auto maximumParameter=static_cast<double>(count-degree);
            knots.assign(count+degree+1,maximumParameter);
            std::fill_n(knots.begin(),degree+1,0.0);
            for(std::size_t index=degree+1;index<count;++index)
                knots[index]=index-degree;
            firstKnots=std::vector<double>(knots.begin()+1,knots.end()-1);
            secondKnots=std::vector<double>(firstKnots.begin()+1,firstKnots.end()-1);
            thirdKnots=std::vector<double>(secondKnots.begin()+1,secondKnots.end()-1);
            first.resize(count-1);
            second.resize(count-2);
            third.resize(count-3);
            for(std::size_t index=0;index<first.size();++index)
                first[index]=scaled(subtract(spline.controls[index+1],spline.controls[index]),
                    static_cast<double>(degree)/(knots[index+degree+1]-knots[index+1]));
            for(std::size_t index=0;index<second.size();++index)
                second[index]=scaled(subtract(first[index+1],first[index]),
                    static_cast<double>(degree-1)
                        /(firstKnots[index+degree]-firstKnots[index+1]));
            for(std::size_t index=0;index<third.size();++index)
                third[index]=scaled(subtract(second[index+1],second[index]),
                    static_cast<double>(degree-2)
                        /(secondKnots[index+degree-1]-secondKnots[index+1]));
        }

        DifferentialSample at(const double parameter) const {
            DifferentialSample result;
            result.position=evaluate(spline.controls,knots,spline.degree,parameter);
            const auto r1=evaluate(first,firstKnots,spline.degree-1,parameter);
            const auto r2=evaluate(second,secondKnots,spline.degree-2,parameter);
            const auto r3=evaluate(third,thirdKnots,spline.degree-3,parameter);
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

    struct SourceSegment {
        const Primitive *primitive=nullptr;
        double from=0.0;
        double to=0.0;
        double length=0.0;
    };

    struct SourceComposite {
        std::vector<SourceSegment> segments;
        double length=0.0;

        Point positionAtDistance(const double requested) const {
            if(segments.empty()) throw std::runtime_error("source composite is empty");
            const auto distance=std::clamp(requested,0.0,length);
            auto segmentStart=0.0;
            for(const auto &segment:segments) {
                if(distance<=segmentStart+segment.length||&segment==&segments.back()) {
                    const auto local=segment.length>0.0
                        ?(distance-segmentStart)/segment.length:0.0;
                    return segment.primitive->positionAtDistance(
                        std::lerp(segment.from,segment.to,std::clamp(local,0.0,1.0)));
                }
                segmentStart+=segment.length;
            }
            return segments.back().primitive->positionAtDistance(segments.back().to);
        }
    };

    double closestPrimitiveDistance(const Primitive &primitive,const Point &target,
                                    const bool preferLater) {
        const auto primitiveLength=primitive.length();
        if(!primitive.arc) {
            const auto delta=subtract(primitive.end,primitive.start);
            const auto denominator=dot(delta,delta);
            if(denominator<=1e-30) return 0.0;
            return primitiveLength*std::clamp(
                dot(subtract(target,primitive.start),delta)/denominator,0.0,1.0);
        }
        constexpr unsigned COARSE_INTERVALS=256;
        auto best=0U;
        auto bestSquared=std::numeric_limits<double>::infinity();
        for(unsigned sample=0;sample<=COARSE_INTERVALS;++sample) {
            const auto distance=primitiveLength*sample/COARSE_INTERVALS;
            const auto delta=subtract(primitive.positionAtDistance(distance),target);
            const auto squared=dot(delta,delta);
            if(squared<bestSquared
               ||(squared==bestSquared&&((preferLater&&sample>best)
                                         ||(!preferLater&&sample<best)))) {
                best=sample;
                bestSquared=squared;
            }
        }
        auto lower=primitiveLength*(best==0?0:best-1)/COARSE_INTERVALS;
        auto upper=primitiveLength*(best==COARSE_INTERVALS?COARSE_INTERVALS:best+1)
            /COARSE_INTERVALS;
        const auto squaredDistance=[&](const double distance) {
            const auto delta=subtract(primitive.positionAtDistance(distance),target);
            return dot(delta,delta);
        };
        constexpr double GOLDEN=0.6180339887498948482;
        auto left=upper-GOLDEN*(upper-lower);
        auto right=lower+GOLDEN*(upper-lower);
        auto leftValue=squaredDistance(left);
        auto rightValue=squaredDistance(right);
        for(unsigned iteration=0;iteration<64;++iteration) {
            if(leftValue<=rightValue) {
                upper=right;
                right=left;
                rightValue=leftValue;
                left=upper-GOLDEN*(upper-lower);
                leftValue=squaredDistance(left);
            } else {
                lower=left;
                left=right;
                leftValue=rightValue;
                right=lower+GOLDEN*(upper-lower);
                rightValue=squaredDistance(right);
            }
        }
        return std::midpoint(lower,upper);
    }

    SourceComposite sourceCompositeFor(const Snapshot &snapshot,const Spline &spline) {
        if(spline.firstSource>spline.lastSource
           ||spline.lastSource>=snapshot.primitives.size())
            throw std::runtime_error("spline source range is outside the captured primitives");
        SourceComposite result;
        result.segments.reserve(spline.lastSource-spline.firstSource+1);
        for(auto source=spline.firstSource;source<=spline.lastSource;++source) {
            const auto &primitive=snapshot.primitives[source];
            if(primitive.id!=source)
                throw std::runtime_error("captured primitive IDs are not dense and ordered");
            auto from=0.0;
            auto to=primitive.length();
            if(source==spline.firstSource)
                from=closestPrimitiveDistance(primitive,spline.controls.front(),true);
            if(source==spline.lastSource)
                to=closestPrimitiveDistance(primitive,spline.controls.back(),false);
            if(to+1e-12<from)
                throw std::runtime_error("captured spline endpoints reverse their source primitive");
            const auto segmentLength=std::max(0.0,to-from);
            result.segments.push_back({&primitive,from,to,segmentLength});
            result.length+=segmentLength;
        }
        if(result.length<=1e-12)
            throw std::runtime_error("captured spline has an empty source primitive interval");
        const auto endpointError=std::max(
            length(subtract(result.positionAtDistance(0.0),spline.controls.front())),
            length(subtract(result.positionAtDistance(result.length),spline.controls.back())));
        if(endpointError>1e-7)
            throw std::runtime_error("could not associate captured spline endpoints with source primitives");
        return result;
    }

    SourceComposite completePrimitiveRange(const Snapshot &snapshot,
            const std::size_t first,const std::size_t last) {
        if(first>last||last>=snapshot.primitives.size())
            throw std::runtime_error("source primitive range is invalid");
        SourceComposite result;
        result.segments.reserve(last-first+1);
        for(auto source=first;source<=last;++source) {
            const auto &primitive=snapshot.primitives[source];
            if(primitive.id!=source)
                throw std::runtime_error("captured primitive IDs are not dense and ordered");
            const auto segmentLength=primitive.length();
            result.segments.push_back({&primitive,0.0,segmentLength,segmentLength});
            result.length+=segmentLength;
        }
        return result;
    }

    Spline halfEntityControlSpline(const Snapshot &snapshot,const Spline &original) {
        if(original.degree!=3||original.controls.size()<=6
           ||original.lastSource<=original.firstSource+1)
            return original;
        const auto interiorEntityCount=original.lastSource-original.firstSource-1;
        const auto interiorControlCount=(interiorEntityCount+1)/2;
        const auto interior=completePrimitiveRange(
            snapshot,original.firstSource+1,original.lastSource-1);
        Spline candidate=original;
        candidate.controls.clear();
        candidate.controls.reserve(interiorControlCount+6);
        candidate.controls.insert(candidate.controls.end(),
            original.controls.begin(),original.controls.begin()+3);
        for(std::size_t control=0;control<interiorControlCount;++control) {
            const auto distance=interior.length*(static_cast<double>(control)+0.5)
                /static_cast<double>(interiorControlCount);
            candidate.controls.push_back(interior.positionAtDistance(distance));
        }
        candidate.controls.insert(candidate.controls.end(),
            original.controls.end()-3,original.controls.end());
        candidate.boundaries.clear();
        return candidate;
    }

    struct OrderedDeviation {
        double maximum=0.0;
        double candidateLength=0.0;
    };

    OrderedDeviation orderedSourceDeviation(const Spline &candidate,
            const SourceComposite &source,const unsigned samplesPerSpan) {
        if(samplesPerSpan==0) throw std::runtime_error("ordered deviation needs samples");
        const SplineEvaluator evaluator(candidate);
        const auto spans=candidate.controls.size()-candidate.degree;
        const auto sampleCount=spans*samplesPerSpan;
        std::vector<Point> positions;
        std::vector<double> distances;
        positions.reserve(sampleCount+1);
        distances.reserve(sampleCount+1);
        for(std::size_t sample=0;sample<=sampleCount;++sample) {
            const auto parameter=static_cast<double>(spans)*sample/sampleCount;
            positions.push_back(evaluator.at(parameter).position);
            distances.push_back(sample==0?0.0:distances.back()
                +length(subtract(positions.back(),positions[positions.size()-2])));
        }
        OrderedDeviation result{.candidateLength=distances.back()};
        if(result.candidateLength<=1e-15) {
            result.maximum=std::numeric_limits<double>::infinity();
            return result;
        }
        for(std::size_t sample=0;sample<positions.size();++sample) {
            const auto sourceDistance=source.length*distances[sample]/result.candidateLength;
            result.maximum=std::max(result.maximum,length(subtract(
                positions[sample],source.positionAtDistance(sourceDistance))));
        }
        return result;
    }

    struct OrderedDeviationProxy {
        std::vector<Point> sourcePositions;
        unsigned samplesPerSpan=0;
    };

    OrderedDeviationProxy makeOrderedDeviationProxy(const SourceComposite &source,
            const std::size_t spans,const unsigned samplesPerSpan) {
        OrderedDeviationProxy result{
            .sourcePositions={},
            .samplesPerSpan=samplesPerSpan,
        };
        const auto sampleCount=spans*samplesPerSpan;
        result.sourcePositions.reserve(sampleCount+1);
        for(std::size_t sample=0;sample<=sampleCount;++sample)
            result.sourcePositions.push_back(source.positionAtDistance(
                source.length*sample/sampleCount));
        return result;
    }

    double orderedSourceDeviationProxy(const Spline &candidate,
            const OrderedDeviationProxy &proxy) {
        const SplineEvaluator evaluator(candidate);
        const auto sampleCount=proxy.sourcePositions.size()-1;
        const auto spans=candidate.controls.size()-candidate.degree;
        auto maximum=0.0;
        for(std::size_t sample=0;sample<=sampleCount;++sample) {
            const auto parameter=static_cast<double>(spans)*sample/sampleCount;
            maximum=std::max(maximum,length(subtract(
                evaluator.at(parameter).position,proxy.sourcePositions[sample])));
        }
        return maximum;
    }

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
        const auto spans=spline.controls.size()-spline.degree;
        const auto sampleCount=spans*SAMPLES_PER_SPAN;
        std::vector<FeedProfilePoint> profile;
        profile.reserve(sampleCount+1);
        Point previousPosition{};
        for(std::size_t index=0;index<=sampleCount;++index) {
            const auto parameter=static_cast<double>(spans)*index/sampleCount;
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
        double maximumNormalCurvatureDerivativeParameter=0.0;
        double integratedNormalCurvatureDerivative=0.0;
        double derivativeTraversalCost=0.0;
        double derivativeVelocityCap=0.0;
    };

    Measurement measure(Spline spline,const unsigned samplesPerSpan=32) {
        if(samplesPerSpan==0) throw std::runtime_error("spline measurement needs samples");
        const auto count=spline.controls.size();
        const auto degree=spline.degree;
        const auto maximumParameter=static_cast<double>(count-degree);
        std::vector<double> knots(count+degree+1,maximumParameter);
        std::fill_n(knots.begin(),degree+1,0.0);
        for(std::size_t index=degree+1;index<count;++index) knots[index]=index-degree;
        std::vector<double> firstKnots(knots.begin()+1,knots.end()-1);
        std::vector<double> secondKnots(firstKnots.begin()+1,firstKnots.end()-1);
        std::vector<double> thirdKnots(secondKnots.begin()+1,secondKnots.end()-1);
        std::vector<Point> first(count-1),second(count-2),third(count-3);
        for(std::size_t index=0;index<first.size();++index)
            first[index]=scaled(subtract(spline.controls[index+1],spline.controls[index]),
                static_cast<double>(degree)/(knots[index+degree+1]-knots[index+1]));
        for(std::size_t index=0;index<second.size();++index)
            second[index]=scaled(subtract(first[index+1],first[index]),
                static_cast<double>(degree-1)
                    /(firstKnots[index+degree]-firstKnots[index+1]));
        for(std::size_t index=0;index<third.size();++index)
            third[index]=scaled(subtract(second[index+1],second[index]),
                static_cast<double>(degree-2)
                    /(secondKnots[index+degree-1]-secondKnots[index+1]));

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
        std::vector<double> spanMaximumDerivative(count-degree,0.0);
        std::vector<double> spanLengths(count-degree,0.0);
        for(std::size_t span=0;span+degree<count;++span) {
            double previousSpeed=0.0;
            double previousNormalDerivative=0.0;
            for(unsigned sample=0;sample<=samplesPerSpan;++sample) {
                auto parameter=static_cast<double>(span)
                    +static_cast<double>(sample)/samplesPerSpan;
                if(sample==samplesPerSpan&&span+degree+1<count)
                    parameter=std::nextafter(parameter,static_cast<double>(span));
                const auto r1=evaluate(first,firstKnots,degree-1,parameter);
                const auto r2=evaluate(second,secondKnots,degree-2,parameter);
                const auto r3=evaluate(third,thirdKnots,degree-3,parameter);
                const auto speed=length(r1);
                const auto distanceStep=sample>0
                    ?(previousSpeed+speed)*0.5/samplesPerSpan:0.0;
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
                if(normalDerivative>result.maximumNormalCurvatureDerivative) {
                    result.maximumNormalCurvatureDerivative=normalDerivative;
                    result.maximumNormalCurvatureDerivativeParameter=parameter;
                }
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
        if(best.spline.degree!=3||best.spline.controls.size()<=6
           ||best.length<0.35||best.length>0.8
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

    struct PhysicalJerkOptimization {
        Measurement measurement;
        double originalSourceDeviation=0.0;
        double originalLength=0.0;
        double maximumSourceDeviation=0.0;
        double candidateLength=0.0;
        std::size_t candidateEvaluations=0;
        std::size_t acceptedCandidates=0;
        bool budgetExhausted=false;
        bool retained=false;
        bool finalDeviationRejected=false;
        bool finalCurvatureRejected=false;
        bool finalObjectiveRejected=false;
        double computationSeconds=0.0;
    };

    PhysicalJerkOptimization optimizePhysicalJerk(const Measurement &original,
            const SourceComposite &source,const double programmedScale,
            const double maximumCurveDeviationInP) {
        constexpr std::size_t MAXIMUM_CANDIDATE_EVALUATIONS=100000;
        const auto started=std::chrono::steady_clock::now();
        const auto originalDeviation=orderedSourceDeviation(original.spline,source,256);
        PhysicalJerkOptimization best{
            .measurement=original,
            .originalSourceDeviation=originalDeviation.maximum,
            .originalLength=originalDeviation.candidateLength,
            .maximumSourceDeviation=originalDeviation.maximum,
            .candidateLength=originalDeviation.candidateLength,
        };
        if(original.spline.degree!=3||original.spline.controls.size()<=6
           ||!std::isfinite(maximumCurveDeviationInP)
           ||maximumCurveDeviationInP<=0.0
           ||original.maximumNormalCurvatureDerivative<=0.0
           ||original.integratedNormalCurvatureDerivative<=0.0) {
            best.computationSeconds=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-started).count();
            return best;
        }
        const auto deviationLimit=programmedScale*maximumCurveDeviationInP;
        const auto deviationProxy=makeOrderedDeviationProxy(
            source,original.spline.controls.size()-original.spline.degree,24);
        const auto score=[&](const Measurement &measurement) {
            const auto peak=measurement.maximumNormalCurvatureDerivative
                /original.maximumNormalCurvatureDerivative;
            const auto integral=measurement.integratedNormalCurvatureDerivative
                /original.integratedNormalCurvatureDerivative;
            return 0.65*peak+0.35*integral
                +4.0*std::max(0.0,peak-1.0)
                +2.0*std::max(0.0,integral-1.0);
        };
        auto bestScore=score(measure(original.spline,24));
        const SplineEvaluator originalEvaluator(original.spline);
        const auto startGeometry=originalEvaluator.at(0.0);
        const auto endGeometry=originalEvaluator.at(
            static_cast<double>(original.spline.controls.size()-original.spline.degree));
        const auto consider=[&](Spline candidateSpline) {
            if(best.candidateEvaluations>=MAXIMUM_CANDIDATE_EVALUATIONS) {
                best.budgetExhausted=true;
                return false;
            }
            ++best.candidateEvaluations;
            const auto deviation=orderedSourceDeviationProxy(candidateSpline,deviationProxy);
            if(deviation>0.98*deviationLimit*(1.0+1e-12)) return false;
            auto candidate=measure(std::move(candidateSpline),24);
            if(candidate.maximumCurvature>original.maximumCurvature*1.02) return false;
            const auto candidateScore=score(candidate);
            if(candidateScore>=bestScore*(1.0-1e-10)) return false;
            best.measurement=std::move(candidate);
            best.maximumSourceDeviation=deviation;
            ++best.acceptedCandidates;
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
        best.measurement=measure(std::move(best.measurement.spline));
        auto verifiedDeviation=orderedSourceDeviation(best.measurement.spline,source,256);
        best.maximumSourceDeviation=verifiedDeviation.maximum;
        best.candidateLength=verifiedDeviation.candidateLength;
        if(best.acceptedCandidates>0) {
            best.finalDeviationRejected=
                best.maximumSourceDeviation>deviationLimit*(1.0+1e-9);
            best.finalCurvatureRejected=best.measurement.maximumCurvature
                >original.maximumCurvature*1.02*(1.0+1e-9);
            best.finalObjectiveRejected=
                best.measurement.maximumNormalCurvatureDerivative
                    >=original.maximumNormalCurvatureDerivative
                ||best.measurement.integratedNormalCurvatureDerivative
                    >=original.integratedNormalCurvatureDerivative;
            best.retained=!best.finalDeviationRejected&&!best.finalCurvatureRejected
                &&!best.finalObjectiveRejected;
        }
        if(!best.retained) {
            best.measurement=original;
            best.maximumSourceDeviation=originalDeviation.maximum;
            best.candidateLength=originalDeviation.candidateLength;
        }
        best.computationSeconds=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
        return best;
    }

    struct EndpointErrors {
        double position=0.0;
        double tangent=0.0;
        double curvature=0.0;
    };

    EndpointErrors endpointErrors(const Spline &original,const Spline &candidate) {
        const SplineEvaluator originalEvaluator(original);
        const SplineEvaluator candidateEvaluator(candidate);
        const auto maximumParameter=static_cast<double>(
            original.controls.size()-original.degree);
        const auto originalStart=originalEvaluator.at(0.0);
        const auto candidateStart=candidateEvaluator.at(0.0);
        const auto originalEnd=originalEvaluator.at(maximumParameter);
        const auto candidateEnd=candidateEvaluator.at(maximumParameter);
        return {
            .position=std::max(
                length(subtract(originalStart.position,candidateStart.position)),
                length(subtract(originalEnd.position,candidateEnd.position))),
            .tangent=std::max(
                length(subtract(originalStart.tangent,candidateStart.tangent)),
                length(subtract(originalEnd.tangent,candidateEnd.tangent))),
            .curvature=std::max(
                length(subtract(originalStart.curvature,candidateStart.curvature)),
                length(subtract(originalEnd.curvature,candidateEnd.curvature))),
        };
    }

    void writeOptimizationReport(const Snapshot &snapshot,const double programmedScale,
            const double maximumSourceDeviationInP,const std::filesystem::path &outputPath) {
        if(!std::isfinite(programmedScale)||programmedScale<=0.0)
            throw std::runtime_error("programmed scale must be positive and finite");
        if(!std::isfinite(maximumSourceDeviationInP)||maximumSourceDeviationInP<=0.0)
            throw std::runtime_error("maximum source deviation must be positive and finite");
        std::ofstream output(outputPath,std::ios::trunc);
        output.precision(17);
        output<<"id,horizon,first_source,last_source,controls,spans,eligible,improved,"
            "source_length,original_length,optimized_length,length_change,"
            "original_max_curvature,optimized_max_curvature,curvature_ratio,"
            "original_max_normal_qs,optimized_max_normal_qs,normal_qs_ratio,"
            "jerk_limited_speed_gain,original_integrated_normal_qs,"
            "optimized_integrated_normal_qs,integrated_normal_qs_ratio,"
            "original_source_deviation,optimized_source_deviation,deviation_limit,"
            "endpoint_position_error,endpoint_tangent_error,endpoint_curvature_error,"
            "candidate_evaluations,accepted_candidates,budget_exhausted,"
            "retained,final_deviation_rejected,final_curvature_rejected,"
            "final_objective_rejected,"
            "optimization_seconds\n";
        auto improvedCount=std::size_t{0};
        auto eligibleCount=std::size_t{0};
        auto regressionCount=std::size_t{0};
        auto maximumGain=1.0;
        auto totalSeconds=0.0;
        for(const auto &spline:snapshot.splines) {
            // Six-control splines are ordinary two-entity junction blends. This
            // experiment targets only the variable-control short-entity splines.
            if(spline.controls.size()<=6) continue;
            const auto source=sourceCompositeFor(snapshot,spline);
            const auto original=measure(spline);
            const auto optimized=optimizePhysicalJerk(
                original,source,programmedScale,maximumSourceDeviationInP);
            const auto errors=endpointErrors(original.spline,optimized.measurement.spline);
            const auto eligible=true;
            const auto improved=optimized.retained
                &&optimized.measurement.maximumNormalCurvatureDerivative
                <original.maximumNormalCurvatureDerivative*(1.0-1e-10)
                &&optimized.measurement.integratedNormalCurvatureDerivative
                <original.integratedNormalCurvatureDerivative*(1.0-1e-10);
            const auto curvatureRatio=original.maximumCurvature>0.0
                ?optimized.measurement.maximumCurvature/original.maximumCurvature:1.0;
            const auto peakRatio=original.maximumNormalCurvatureDerivative>0.0
                ?optimized.measurement.maximumNormalCurvatureDerivative
                    /original.maximumNormalCurvatureDerivative:1.0;
            const auto integralRatio=original.integratedNormalCurvatureDerivative>0.0
                ?optimized.measurement.integratedNormalCurvatureDerivative
                    /original.integratedNormalCurvatureDerivative:1.0;
            const auto speedGain=peakRatio>0.0?std::cbrt(1.0/peakRatio):1.0;
            const auto deviationLimit=programmedScale*maximumSourceDeviationInP;
            const auto regressed=improved&&(optimized.maximumSourceDeviation>
                    deviationLimit*(1.0+1e-9)
                ||curvatureRatio>1.02*(1.0+1e-9)
                ||errors.position>1e-10||errors.tangent>1e-10||errors.curvature>1e-8);
            eligibleCount+=eligible;
            improvedCount+=improved;
            regressionCount+=regressed;
            maximumGain=std::max(maximumGain,speedGain);
            totalSeconds+=optimized.computationSeconds;
            output<<spline.id<<','<<spline.horizon<<','<<spline.firstSource<<','
                <<spline.lastSource<<','<<spline.controls.size()<<','
                <<spline.controls.size()-spline.degree<<','<<eligible<<','<<improved<<','
                <<source.length<<','<<optimized.originalLength<<','
                <<optimized.candidateLength<<','
                <<optimized.candidateLength-optimized.originalLength<<','
                <<original.maximumCurvature<<','<<optimized.measurement.maximumCurvature<<','
                <<curvatureRatio<<','<<original.maximumNormalCurvatureDerivative<<','
                <<optimized.measurement.maximumNormalCurvatureDerivative<<','<<peakRatio<<','
                <<speedGain<<','<<original.integratedNormalCurvatureDerivative<<','
                <<optimized.measurement.integratedNormalCurvatureDerivative<<','
                <<integralRatio<<','<<optimized.originalSourceDeviation<<','
                <<optimized.maximumSourceDeviation<<','<<deviationLimit<<','
                <<errors.position<<','<<errors.tangent<<','<<errors.curvature<<','
                <<optimized.candidateEvaluations<<','<<optimized.acceptedCandidates<<','
                <<optimized.budgetExhausted<<','<<optimized.retained<<','
                <<optimized.finalDeviationRejected<<','
                <<optimized.finalCurvatureRejected<<','
                <<optimized.finalObjectiveRejected<<','
                <<optimized.computationSeconds<<'\n';
        }
        output.flush();
        if(!output) throw std::runtime_error("incomplete optimization report write");
        std::cout<<"optimized "<<eligibleCount<<" variable-control short-entity splines from "
            <<snapshot.splines.size()<<" captured splines ("<<improvedCount<<" improved, "
            <<regressionCount<<" regressions) into "<<outputPath.string()
            <<"; maximum theoretical local jerk-limited speed gain "<<maximumGain
            <<"; optimizer time "<<totalSeconds<<" s\n";
    }

    void writeHalfEntityControlReport(const Snapshot &snapshot,
            const std::filesystem::path &outputPath) {
        std::ofstream output(outputPath,std::ios::trunc);
        output.precision(17);
        output<<"id,horizon,first_source,last_source,interior_entities,"
            "original_controls,half_entity_controls,original_spans,half_entity_spans,"
            "original_max_curvature,half_entity_max_curvature,curvature_ratio,"
            "original_max_qs,half_entity_max_qs,max_qs_ratio,"
            "original_max_qs_parameter,half_entity_max_qs_parameter,"
            "original_max_normal_qs,half_entity_max_normal_qs,normal_qs_ratio,"
            "original_max_normal_qs_parameter,half_entity_max_normal_qs_parameter,"
            "half_entity_peak_distance_to_knot,"
            "jerk_limited_speed_gain,original_integrated_normal_qs,"
            "half_entity_integrated_normal_qs,integrated_normal_qs_ratio,"
            "original_min_parameter_speed,half_entity_min_parameter_speed,"
            "source_length,original_length,half_entity_length,length_change,"
            "original_source_deviation,half_entity_source_deviation,"
            "endpoint_position_error,endpoint_tangent_error,endpoint_curvature_error\n";
        auto count=std::size_t{0};
        auto lowerPeakCount=std::size_t{0};
        auto lowerNormalPeakCount=std::size_t{0};
        auto lowerIntegralCount=std::size_t{0};
        auto maximumSpeedGain=0.0;
        for(const auto &spline:snapshot.splines) {
            if(spline.controls.size()<=6||spline.lastSource<=spline.firstSource+1)
                continue;
            const auto source=sourceCompositeFor(snapshot,spline);
            const auto candidateSpline=halfEntityControlSpline(snapshot,spline);
            const auto original=measure(spline);
            const auto candidate=measure(candidateSpline);
            const auto originalDeviation=orderedSourceDeviation(spline,source,256);
            const auto candidateDeviation=orderedSourceDeviation(candidateSpline,source,256);
            const auto errors=endpointErrors(spline,candidateSpline);
            const auto curvatureRatio=original.maximumCurvature>0.0
                ?candidate.maximumCurvature/original.maximumCurvature:1.0;
            const auto derivativeRatio=original.maximumCurvatureDerivative>0.0
                ?candidate.maximumCurvatureDerivative
                    /original.maximumCurvatureDerivative:1.0;
            const auto normalRatio=original.maximumNormalCurvatureDerivative>0.0
                ?candidate.maximumNormalCurvatureDerivative
                    /original.maximumNormalCurvatureDerivative:1.0;
            const auto integralRatio=original.integratedNormalCurvatureDerivative>0.0
                ?candidate.integratedNormalCurvatureDerivative
                    /original.integratedNormalCurvatureDerivative:1.0;
            const auto speedGain=normalRatio>0.0
                ?std::cbrt(1.0/normalRatio):std::numeric_limits<double>::infinity();
            ++count;
            lowerPeakCount+=derivativeRatio<1.0;
            lowerNormalPeakCount+=normalRatio<1.0;
            lowerIntegralCount+=integralRatio<1.0;
            maximumSpeedGain=std::max(maximumSpeedGain,speedGain);
            output<<spline.id<<','<<spline.horizon<<','<<spline.firstSource<<','
                <<spline.lastSource<<','
                <<spline.lastSource-spline.firstSource-1<<','
                <<spline.controls.size()<<','<<candidateSpline.controls.size()<<','
                <<spline.controls.size()-spline.degree<<','
                <<candidateSpline.controls.size()-candidateSpline.degree<<','
                <<original.maximumCurvature<<','<<candidate.maximumCurvature<<','
                <<curvatureRatio<<','<<original.maximumCurvatureDerivative<<','
                <<candidate.maximumCurvatureDerivative<<','<<derivativeRatio<<','
                <<original.maximumCurvatureDerivativeParameter<<','
                <<candidate.maximumCurvatureDerivativeParameter<<','
                <<original.maximumNormalCurvatureDerivative<<','
                <<candidate.maximumNormalCurvatureDerivative<<','<<normalRatio<<','
                <<original.maximumNormalCurvatureDerivativeParameter<<','
                <<candidate.maximumNormalCurvatureDerivativeParameter<<','
                <<std::abs(candidate.maximumNormalCurvatureDerivativeParameter
                    -std::round(candidate.maximumNormalCurvatureDerivativeParameter))<<','
                <<speedGain<<','<<original.integratedNormalCurvatureDerivative<<','
                <<candidate.integratedNormalCurvatureDerivative<<','<<integralRatio<<','
                <<original.minimumParameterSpeed<<','<<candidate.minimumParameterSpeed<<','
                <<source.length<<','<<originalDeviation.candidateLength<<','
                <<candidateDeviation.candidateLength<<','
                <<candidateDeviation.candidateLength-originalDeviation.candidateLength<<','
                <<originalDeviation.maximum<<','<<candidateDeviation.maximum<<','
                <<errors.position<<','<<errors.tangent<<','<<errors.curvature<<'\n';
        }
        output.flush();
        if(!output) throw std::runtime_error("incomplete half-entity control report write");
        std::cout<<"reconstructed "<<count
            <<" variable-control short-entity splines with one interior control per two "
            <<"entities (rounded up) into "<<outputPath.string()<<"; lower total peak q_s="
            <<lowerPeakCount<<", lower normal peak q_s="<<lowerNormalPeakCount
            <<", lower integrated normal q_s="<<lowerIntegralCount
            <<", maximum theoretical local jerk-limited speed gain="
            <<maximumSpeedGain<<'\n';
    }

    struct CurvatureDerivativeProfilePoint {
        double distanceFraction=0.0;
        double normalDerivative=0.0;
    };

    std::vector<CurvatureDerivativeProfilePoint> curvatureDerivativeProfile(
            const Spline &spline) {
        constexpr std::size_t SAMPLES_PER_SPAN=128;
        const auto spans=spline.controls.size()-spline.degree;
        const auto sampleCount=spans*SAMPLES_PER_SPAN;
        const SplineEvaluator evaluator(spline);
        std::vector<CurvatureDerivativeProfilePoint> result;
        result.reserve(sampleCount+1);
        Point previous{};
        for(std::size_t sample=0;sample<=sampleCount;++sample) {
            const auto parameter=static_cast<double>(spans)*sample/sampleCount;
            const auto geometry=evaluator.at(parameter);
            const auto tangential=dot(geometry.tangent,geometry.curvatureDerivative);
            const auto normal=length(subtract(geometry.curvatureDerivative,
                scaled(geometry.tangent,tangential)));
            const auto distance=sample==0?0.0:result.back().distanceFraction
                +length(subtract(geometry.position,previous));
            result.push_back({distance,normal});
            previous=geometry.position;
        }
        const auto total=result.back().distanceFraction;
        for(auto &point:result) point.distanceFraction/=total;
        return result;
    }

    void writeHalfEntityProfile(const Snapshot &snapshot,const std::size_t splineId,
            const std::filesystem::path &basePath) {
        const auto found=std::ranges::find(snapshot.splines,splineId,&Spline::id);
        if(found==snapshot.splines.end()) throw std::runtime_error("requested spline ID not found");
        const auto candidate=halfEntityControlSpline(snapshot,*found);
        const auto originalProfile=curvatureDerivativeProfile(*found);
        const auto candidateProfile=curvatureDerivativeProfile(candidate);
        const auto originalPeak=std::ranges::max(originalProfile,{},
            &CurvatureDerivativeProfilePoint::normalDerivative).normalDerivative;
        const auto candidatePeak=std::ranges::max(candidateProfile,{},
            &CurvatureDerivativeProfilePoint::normalDerivative).normalDerivative;
        const auto maximum=std::max(originalPeak,candidatePeak);

        auto csvPath=basePath;
        csvPath.replace_extension("csv");
        std::ofstream csv(csvPath,std::ios::trunc);
        csv.precision(17);
        csv<<"series,distance_fraction,normal_curvature_derivative\n";
        for(const auto &point:originalProfile)
            csv<<"production,"<<point.distanceFraction<<','<<point.normalDerivative<<'\n';
        for(const auto &point:candidateProfile)
            csv<<"half_entity,"<<point.distanceFraction<<','<<point.normalDerivative<<'\n';
        csv.flush();
        if(!csv) throw std::runtime_error("incomplete half-entity profile CSV write");

        auto svgPath=basePath;
        svgPath.replace_extension("svg");
        std::ofstream svg(svgPath,std::ios::trunc);
        constexpr double WIDTH=1200.0,HEIGHT=660.0,LEFT=105.0,RIGHT=35.0;
        constexpr double TOP=85.0,BOTTOM=95.0;
        const auto plotWidth=WIDTH-LEFT-RIGHT;
        const auto plotHeight=HEIGHT-TOP-BOTTOM;
        const auto x=[&](const double fraction) { return LEFT+plotWidth*fraction; };
        const auto y=[&](const double derivative) {
            return TOP+plotHeight*(1.0-derivative/maximum);
        };
        svg<<"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1200\" height=\"660\" "
            "viewBox=\"0 0 1200 660\"><rect width=\"1200\" height=\"660\" fill=\"white\"/>"
            "<style>text{font-family:Segoe UI,Arial,sans-serif;fill:#222}.grid{stroke:#ddd;stroke-width:1}"
            ".axis{stroke:#222;stroke-width:1.4}.production{fill:none;stroke:#222;stroke-width:1.7}"
            ".half{fill:none;stroke:#d62728;stroke-width:1.8}</style>";
        svg<<"<text x=\""<<LEFT<<"\" y=\"30\" font-size=\"22\" font-weight=\"600\">"
            "Slow trochoid spline "<<splineId<<": normal curvature derivative</text>"
            "<text x=\""<<LEFT<<"\" y=\"57\" font-size=\"14\">Production peak "
            <<originalPeak<<"; half-entity peak "<<candidatePeak<<" (ratio "
            <<candidatePeak/originalPeak<<")</text>";
        for(unsigned grid=0;grid<=5;++grid) {
            const auto value=maximum*grid/5.0;
            const auto py=y(value);
            svg<<"<line class=\"grid\" x1=\""<<LEFT<<"\" y1=\""<<py
                <<"\" x2=\""<<WIDTH-RIGHT<<"\" y2=\""<<py<<"\"/>"
                "<text x=\""<<LEFT-10<<"\" y=\""<<py+5
                <<"\" font-size=\"12\" text-anchor=\"end\">"<<value<<"</text>";
            const auto fraction=static_cast<double>(grid)/5.0;
            const auto px=x(fraction);
            svg<<"<line class=\"grid\" x1=\""<<px<<"\" y1=\""<<TOP
                <<"\" x2=\""<<px<<"\" y2=\""<<TOP+plotHeight<<"\"/>"
                "<text x=\""<<px<<"\" y=\""<<TOP+plotHeight+24
                <<"\" font-size=\"12\" text-anchor=\"middle\">"<<fraction<<"</text>";
        }
        svg<<"<line class=\"axis\" x1=\""<<LEFT<<"\" y1=\""<<TOP
            <<"\" x2=\""<<LEFT<<"\" y2=\""<<TOP+plotHeight<<"\"/>"
            "<line class=\"axis\" x1=\""<<LEFT<<"\" y1=\""<<TOP+plotHeight
            <<"\" x2=\""<<WIDTH-RIGHT<<"\" y2=\""<<TOP+plotHeight<<"\"/>";
        const auto polyline=[&](const std::string_view css,
                                const std::vector<CurvatureDerivativeProfilePoint> &profile) {
            svg<<"<polyline class=\""<<css<<"\" points=\"";
            for(const auto &point:profile)
                svg<<x(point.distanceFraction)<<','<<y(point.normalDerivative)<<' ';
            svg<<"\"/>";
        };
        polyline("production",originalProfile);
        polyline("half",candidateProfile);
        svg<<"<text x=\"600\" y=\"625\" font-size=\"15\" text-anchor=\"middle\">"
            "Normalized distance along spline</text>"
            "<text x=\"22\" y=\"330\" font-size=\"15\" text-anchor=\"middle\" "
            "transform=\"rotate(-90 22 330)\">Normal curvature derivative</text>"
            "<line class=\"production\" x1=\"760\" y1=\"620\" x2=\"800\" y2=\"620\"/>"
            "<text x=\"810\" y=\"625\" font-size=\"13\">production</text>"
            "<line class=\"half\" x1=\"930\" y1=\"620\" x2=\"970\" y2=\"620\"/>"
            "<text x=\"980\" y=\"625\" font-size=\"13\">half-entity controls</text></svg>";
        svg.flush();
        if(!svg) throw std::runtime_error("incomplete half-entity profile SVG write");
        std::cout<<"wrote half-entity curvature-derivative comparison to "
            <<csvPath.string()<<" and "<<svgPath.string()<<'\n';
    }

    Snapshot readSnapshot(std::istream &input) {
        std::string word;
        input>>word;
        const auto version2=word=="ngc_spline_geometry_v2";
        if(!version2&&word!="ngc_spline_geometry_v1")
            throw std::runtime_error("unsupported snapshot");
        std::size_t primitiveCount=0;
        input>>word>>primitiveCount;
        if(word!="primitive_count") throw std::runtime_error("malformed primitive count");
        Snapshot result;
        result.primitives.reserve(primitiveCount);
        for(std::size_t primitive=0;primitive<primitiveCount;++primitive) {
            Primitive value;
            input>>word>>value.id>>value.feed;
            if(word!="line"&&word!="arc")
                throw std::runtime_error("malformed captured primitive");
            value.arc=word=="arc";
            for(auto &coordinate:value.start) input>>coordinate;
            for(auto &coordinate:value.end) input>>coordinate;
            if(value.arc) {
                for(auto &coordinate:value.center) input>>coordinate;
                for(auto &coordinate:value.axis) input>>coordinate;
            }
            value.prepare();
            result.primitives.push_back(std::move(value));
        }
        std::size_t splineCount=0;
        input>>word>>splineCount;
        if(word!="spline_count") throw std::runtime_error("malformed spline count");
        result.splines.reserve(splineCount);
        for(std::size_t index=0;index<splineCount;++index) {
            Spline spline;
            std::size_t controlCount=0,boundaryCount=0;
            input>>word>>spline.id>>spline.horizon>>spline.firstSource>>spline.lastSource;
            if(version2) input>>spline.degree;
            input>>controlCount>>boundaryCount;
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
            result.splines.push_back(std::move(spline));
        }
        if(!input) throw std::runtime_error("incomplete geometry snapshot");
        return result;
    }

    Snapshot readSnapshot(const std::filesystem::path &path) {
        if(path=="--self-test-fixture") {
            std::istringstream input{R"SNAPSHOT(ngc_spline_geometry_v1
primitive_count 3
line 0 80 0 0 0 0 0 0 1 0 0 0 0 0
arc 1 80 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1
line 2 80 0 1 0 0 0 0 0 2 0 0 0 0
spline_count 1
spline 0 1 0 2 7 2
control 0.5 0 0 0 0 0
control 0.7 0 0 0 0 0
control 0.9 0.1 0 0 0 0
control 0.9 0.5 0 0 0 0
control 0.5 0.9 0 0 0 0
control 0 1.3 0 0 0 0
control 0 1.5 0 0 0 0
boundaries 0 2.5707963267948966
)SNAPSHOT"};
            return readSnapshot(input);
        }
        std::ifstream input(path);
        if(!input) throw std::runtime_error("could not open geometry snapshot");
        return readSnapshot(input);
    }
}

int main(const int argc,char **argv) {
    try {
        if(argc==7&&std::string_view(argv[2])=="--profile-optimized") {
            const auto splineId=std::stoull(argv[3]);
            const auto feedPerMinute=std::stod(argv[4]);
            const auto maximumCurveDeviationInP=std::stod(argv[5]);
            const auto snapshot=readSnapshot(argv[1]);
            const auto found=std::ranges::find(snapshot.splines,splineId,&Spline::id);
            if(found==snapshot.splines.end())
                throw std::runtime_error("requested spline ID not found");
            const auto original=measure(*found);
            const auto source=sourceCompositeFor(snapshot,*found);
            const auto optimized=optimizePhysicalJerk(
                original,source,0.005,maximumCurveDeviationInP);
            writeFeedProfile(optimized.measurement.spline,feedPerMinute,argv[6]);
            const SplineEvaluator originalEvaluator(original.spline);
            const SplineEvaluator optimizedEvaluator(optimized.measurement.spline);
            const auto maximumParameter=static_cast<double>(
                original.spline.controls.size()-original.spline.degree);
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
                <<"; original source deviation "<<optimized.originalSourceDeviation
                <<"; maximum source deviation "<<optimized.maximumSourceDeviation
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
            const auto snapshot=readSnapshot(argv[1]);
            const auto found=std::ranges::find(snapshot.splines,splineId,&Spline::id);
            if(found==snapshot.splines.end())
                throw std::runtime_error("requested spline ID not found");
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
            const auto snapshot=readSnapshot(argv[1]);
            const auto found=std::ranges::find(snapshot.splines,splineId,&Spline::id);
            if(found==snapshot.splines.end())
                throw std::runtime_error("requested spline ID not found");
            writeFeedProfile(*found,feedPerMinute,argv[5]);
            return 0;
        }
        if(argc==6&&std::string_view(argv[2])=="--optimize-all") {
            const auto programmedScale=std::stod(argv[3]);
            const auto maximumSourceDeviationInP=std::stod(argv[4]);
            const auto snapshot=readSnapshot(argv[1]);
            writeOptimizationReport(snapshot,programmedScale,
                maximumSourceDeviationInP,argv[5]);
            return 0;
        }
        if(argc==4&&std::string_view(argv[2])=="--half-entity-controls") {
            const auto snapshot=readSnapshot(argv[1]);
            writeHalfEntityControlReport(snapshot,argv[3]);
            return 0;
        }
        if(argc==5&&std::string_view(argv[2])=="--profile-half-entity") {
            const auto snapshot=readSnapshot(argv[1]);
            writeHalfEntityProfile(snapshot,std::stoull(argv[3]),argv[4]);
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
                "<output-base>\n"
                "       ngc_spline_geometry_analyzer <snapshot.txt> --optimize-all "
                "<programmed-scale> <maximum-source-deviation-in-P> <output.csv>\n"
                "       ngc_spline_geometry_analyzer <snapshot.txt> "
                "--half-entity-controls <output.csv>\n"
                "       ngc_spline_geometry_analyzer <snapshot.txt> "
                "--profile-half-entity <spline-id> <output-base>\n";
            return 2;
        }
        const auto maximumDisplacementInP=argc==4?std::stod(argv[3]):0.20;
        if(!std::isfinite(maximumDisplacementInP)||maximumDisplacementInP<=0.0)
            throw std::runtime_error("maximum displacement must be positive and finite");
        auto snapshot=readSnapshot(argv[1]);
        std::vector<Measurement> measurements;
        measurements.reserve(snapshot.splines.size());
        for(auto &spline:snapshot.splines)
            measurements.push_back(measure(std::move(spline)));
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
                <<spline.controls.size()-spline.degree<<','
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
