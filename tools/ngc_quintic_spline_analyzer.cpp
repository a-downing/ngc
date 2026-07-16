#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
    using Point=std::array<double,6>;
    Point add(const Point&a,const Point&b){Point r{};for(std::size_t i=0;i<6;++i)r[i]=a[i]+b[i];return r;}
    Point sub(const Point&a,const Point&b){Point r{};for(std::size_t i=0;i<6;++i)r[i]=a[i]-b[i];return r;}
    Point mul(const Point&a,double s){Point r{};for(std::size_t i=0;i<6;++i)r[i]=a[i]*s;return r;}
    double dot(const Point&a,const Point&b){return std::inner_product(a.begin(),a.end(),b.begin(),0.0);}
    double norm(const Point&a){return std::sqrt(dot(a,a));}
    Point normalized(Point a){const auto n=norm(a);if(n<=1e-15)throw std::runtime_error("zero vector");return mul(a,1.0/n);}
    std::array<double,3> cross(const Point&a,const Point&b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}

    struct Primitive {
        bool arc=false;
        std::size_t id=0;
        double feed=0.0;
        Point start{},end{};
        std::array<double,3> center{},axis{};

        Point position(double u) const {
            u=std::clamp(u,0.0,1.0);
            if(!arc) {
                Point r{};for(std::size_t i=0;i<6;++i)r[i]=std::lerp(start[i],end[i],u);return r;
            }
            Point axis6{};for(std::size_t i=0;i<3;++i)axis6[i]=axis[i];axis6=normalized(axis6);
            Point c{};for(std::size_t i=0;i<3;++i)c[i]=center[i];
            auto r0=sub(start,c),r1=sub(end,c);
            const auto z0=dot(r0,axis6),z1=dot(r1,axis6);
            r0=sub(r0,mul(axis6,z0));r1=sub(r1,mul(axis6,z1));
            const auto radius0=norm(r0),radius1=norm(r1);
            auto x=mul(r0,1.0/radius0);
            const auto cr=cross(r0,r1);
            auto angle=std::atan2(axis6[0]*cr[0]+axis6[1]*cr[1]+axis6[2]*cr[2],dot(r0,r1));
            if(angle<=1e-12) angle+=2.0*std::acos(-1.0);
            Point axx{};const auto cx=cross(axis6,x);for(std::size_t i=0;i<3;++i)axx[i]=cx[i];
            const auto theta=angle*u;
            auto radial=add(mul(x,std::cos(theta)),mul(axx,std::sin(theta)));
            Point result=add(c,add(mul(radial,std::lerp(radius0,radius1,u)),
                                   mul(axis6,std::lerp(z0,z1,u))));
            for(std::size_t i=3;i<6;++i) result[i]=std::lerp(start[i],end[i],u);
            return result;
        }
        double length(double from=0.0,double to=1.0) const {
            auto total=0.0;auto previous=position(from);
            constexpr unsigned samples=128;
            for(unsigned i=1;i<=samples;++i){const auto current=position(std::lerp(from,to,double(i)/samples));total+=norm(sub(current,previous));previous=current;}
            return total;
        }
        double parameterOf(const Point &target) const {
            auto best=0.0,bestDistance=std::numeric_limits<double>::infinity();
            for(unsigned i=0;i<=1024;++i){const auto u=double(i)/1024;const auto d=norm(sub(position(u),target));if(d<bestDistance){bestDistance=d;best=u;}}
            auto lo=std::max(0.0,best-1.0/1024),hi=std::min(1.0,best+1.0/1024);
            for(unsigned iteration=0;iteration<40;++iteration){const auto a=(2*lo+hi)/3,b=(lo+2*hi)/3;if(norm(sub(position(a),target))<norm(sub(position(b),target)))hi=b;else lo=a;}
            return std::midpoint(lo,hi);
        }
        std::array<Point,3> geometry(double u) const {
            if(!arc) return {normalized(sub(end,start)),Point{},Point{}};
            const auto fullLength=length();
            auto h=std::min(fullLength*0.002,0.001);
            h=std::min(h,std::min(u,1.0-u)*fullLength/2.1);
            if(h<=1e-7) h=std::min(fullLength*0.001,1e-4);
            const auto du=h/fullLength;
            const auto m2=position(u-2*du),m1=position(u-du),p0=position(u),p1=position(u+du),p2=position(u+2*du);
            auto first=mul(sub(p1,m1),1.0/(2*h));first=normalized(first);
            auto second=mul(add(sub(p1,mul(p0,2.0)),m1),1.0/(h*h));
            second=sub(second,mul(first,dot(first,second)));
            auto third=mul(add(sub(p2,mul(p1,2.0)),sub(mul(m1,2.0),m2)),1.0/(2*h*h*h));
            return {first,second,third};
        }
    };

    struct CapturedSpline {std::size_t id=0,first=0,last=0;std::vector<Point> controls;};
    struct Snapshot {std::vector<Primitive> primitives;std::vector<CapturedSpline> splines;};

    Snapshot readSnapshot(const std::filesystem::path &path) {
        std::ifstream in(path);if(!in)throw std::runtime_error("could not open snapshot");
        std::string word;in>>word;if(word!="ngc_spline_geometry_v1")throw std::runtime_error("unsupported snapshot");
        std::size_t count=0;in>>word>>count;Snapshot result;result.primitives.reserve(count);
        for(std::size_t i=0;i<count;++i){Primitive p;in>>word>>p.id>>p.feed;p.arc=word=="arc";for(auto&v:p.start)in>>v;for(auto&v:p.end)in>>v;if(p.arc){for(auto&v:p.center)in>>v;for(auto&v:p.axis)in>>v;}result.primitives.push_back(p);}
        in>>word>>count;result.splines.reserve(count);
        for(std::size_t i=0;i<count;++i){CapturedSpline s;std::size_t horizon=0,n=0,b=0;in>>word>>s.id>>horizon>>s.first>>s.last>>n>>b;s.controls.resize(n);for(auto&c:s.controls){in>>word;for(auto&v:c)in>>v;}in>>word;double ignored=0;for(std::size_t j=0;j<b;++j)in>>ignored;result.splines.push_back(std::move(s));}
        if(!in)throw std::runtime_error("incomplete snapshot");return result;
    }

    struct Segment {const Primitive *primitive=nullptr;double from=0,to=1,length=0;};
    struct Composite {
        std::vector<Segment> segments;double total=0;
        Point position(double distance) const {distance=std::clamp(distance,0.0,total);double start=0;for(const auto&s:segments){if(distance<=start+s.length||&s==&segments.back())return s.primitive->position(std::lerp(s.from,s.to,(distance-start)/s.length));start+=s.length;}return segments.back().primitive->position(segments.back().to);}
    };
    Composite compositeFor(const Snapshot &snapshot,const CapturedSpline &spline){
        Composite c;for(auto source=spline.first;source<=spline.last;++source){const auto&p=snapshot.primitives.at(source);double from=0,to=1;if(source==spline.first)from=p.parameterOf(spline.controls.front());if(source==spline.last)to=p.parameterOf(spline.controls.back());Segment segment{&p,from,to,p.length(from,to)};c.total+=segment.length;c.segments.push_back(segment);}return c;
    }

    struct BSpline {
        unsigned degree=0;std::vector<Point> controls;std::vector<double> knots;
        Point at(double requested) const {
            if(requested<=knots[degree])return controls.front();const auto maximum=knots[controls.size()];if(requested>=maximum)return controls.back();
            auto u=std::clamp(requested,knots[degree],maximum);std::size_t span=degree;while(span+1<controls.size()&&u>=knots[span+1])++span;
            std::vector<Point> work(degree+1);for(unsigned i=0;i<=degree;++i)work[i]=controls[span-degree+i];
            for(unsigned level=1;level<=degree;++level)for(unsigned i=degree;i>=level;--i){const auto k=span-degree+i;const auto alpha=(u-knots[k])/(knots[k+degree-level+1]-knots[k]);work[i]=add(mul(work[i-1],1-alpha),mul(work[i],alpha));}
            return work[degree];
        }
        BSpline derivative() const {BSpline d;d.degree=degree-1;d.knots={knots.begin()+1,knots.end()-1};d.controls.resize(controls.size()-1);for(std::size_t i=0;i<d.controls.size();++i)d.controls[i]=mul(sub(controls[i+1],controls[i]),degree/(knots[i+degree+1]-knots[i+1]));return d;}
    };
    std::vector<double> openKnots(std::size_t controls,unsigned degree){const auto spans=controls-degree;std::vector<double> k(controls+degree+1,double(spans));std::fill_n(k.begin(),degree+1,0.0);for(std::size_t i=degree+1;i<controls;++i)k[i]=double(i-degree);return k;}

    struct Geometry {Point position{},tangent{},curvature{},derivative{};};
    struct SplineEvaluator {
        BSpline spline,first,second,third;
        explicit SplineEvaluator(BSpline value):spline(std::move(value)),first(spline.derivative()),second(first.derivative()),third(second.derivative()){}
        Geometry at(double u) const {Geometry g;g.position=spline.at(u);const auto r1=first.at(u),r2=second.at(u),r3=third.at(u);const auto speed=norm(r1),h=dot(r1,r2),i2=1/(speed*speed),i4=i2*i2,i6=i4*i2;g.tangent=mul(r1,1/speed);for(std::size_t a=0;a<6;++a){g.curvature[a]=r2[a]*i2-r1[a]*h*i4;const auto du=r3[a]*i2-3*r2[a]*h*i4-r1[a]*(dot(r2,r2)+dot(r1,r3))*i4+4*r1[a]*h*h*i6;g.derivative[a]=du/speed;}return g;}
    };

    std::vector<double> derivativeCoefficients(const BSpline &s,unsigned order,bool end){
        const auto n=s.controls.size();std::vector<std::vector<double>> values(n,std::vector<double>(n));for(std::size_t i=0;i<n;++i)values[i][i]=1;auto knots=s.knots;auto degree=s.degree;
        for(unsigned derivative=0;derivative<order;++derivative){std::vector<std::vector<double>> next(values.size()-1,std::vector<double>(n));for(std::size_t i=0;i<next.size();++i){const auto factor=degree/(knots[i+degree+1]-knots[i+1]);for(std::size_t j=0;j<n;++j)next[i][j]=factor*(values[i+1][j]-values[i][j]);}values=std::move(next);knots={knots.begin()+1,knots.end()-1};--degree;}return end?values.back():values.front();
    }
    void imposeDerivative(BSpline &s,unsigned order,const Point &desired,bool end){const auto coefficients=derivativeCoefficients(s,order,end);const auto unknown=end?s.controls.size()-1-order:order;for(std::size_t axis=0;axis<6;++axis){auto residual=desired[axis];for(std::size_t i=0;i<s.controls.size();++i)if(i!=unknown)residual-=coefficients[i]*s.controls[i][axis];s.controls[unknown][axis]=residual/coefficients[unknown];}}

    BSpline quinticFor(const Composite &source,const CapturedSpline &cubic){
        BSpline q;q.degree=5;const auto spans=cubic.controls.size()-3;q.controls.resize(spans+5);q.knots=openKnots(q.controls.size(),q.degree);
        for(std::size_t i=0;i<q.controls.size();++i){auto greville=0.0;for(unsigned j=1;j<=q.degree;++j)greville+=q.knots[i+j];greville/=q.degree;q.controls[i]=source.position(source.total*greville/spans);}
        const auto startSegment=source.segments.front(),endSegment=source.segments.back();const auto startGeometry=startSegment.primitive->geometry(startSegment.from),endGeometry=endSegment.primitive->geometry(endSegment.to);const auto scale=source.total/spans;
        q.controls.front()=source.position(0);q.controls.back()=source.position(source.total);
        imposeDerivative(q,1,mul(startGeometry[0],scale),false);imposeDerivative(q,2,mul(startGeometry[1],scale*scale),false);imposeDerivative(q,3,mul(startGeometry[2],scale*scale*scale),false);
        imposeDerivative(q,1,mul(endGeometry[0],scale),true);imposeDerivative(q,2,mul(endGeometry[1],scale*scale),true);imposeDerivative(q,3,mul(endGeometry[2],scale*scale*scale),true);
        return q;
    }

    struct Measurement {double length=0,maxDerivative=0,maxNormal=0,integralNormal=0,maxCurvature=0,maxDeviation=0;};
    Measurement measure(const BSpline&s,const Composite *source=nullptr,unsigned perSpan=128){Measurement m;const auto spans=s.controls.size()-s.degree;const SplineEvaluator evaluator(s);Point previous=s.at(0);double previousNormal=0;for(std::size_t i=0;i<=spans*perSpan;++i){const auto u=double(i)/perSpan;const auto g=evaluator.at(u);const auto tangentComponent=dot(g.tangent,g.derivative);const auto normal=norm(sub(g.derivative,mul(g.tangent,tangentComponent)));const auto d=norm(g.derivative);m.maxDerivative=std::max(m.maxDerivative,d);m.maxNormal=std::max(m.maxNormal,normal);m.maxCurvature=std::max(m.maxCurvature,norm(g.curvature));if(i){const auto ds=norm(sub(g.position,previous));m.length+=ds;m.integralNormal+=0.5*(previousNormal+normal)*ds;}if(source)m.maxDeviation=std::max(m.maxDeviation,norm(sub(g.position,source->position(source->total*u/spans))));previous=g.position;previousNormal=normal;}return m;}

    BSpline optimizeQuinticPhysicalJerk(BSpline initial,const Composite &source,
                                         double programmedScale,double deviationInP) {
        auto best=std::move(initial);
        const auto original=measure(best,&source,48);
        const auto limit=programmedScale*deviationInP;
        const auto score=[&](const Measurement &m) {
            const auto peak=m.maxNormal/original.maxNormal;
            const auto integral=m.integralNormal/original.integralNormal;
            const auto excess=std::max(0.0,m.maxDeviation/(0.98*limit)-1.0);
            return 0.65*peak+0.35*integral+100.0*excess*excess;
        };
        auto bestMeasurement=original;
        auto bestScore=score(bestMeasurement);
        for(const auto fraction:{0.25,0.125,0.0625,0.03125,0.015625,0.0078125}) {
            const auto step=programmedScale*fraction;
            auto improved=true;
            for(unsigned sweep=0;sweep<3&&improved;++sweep) {
                improved=false;
                for(std::size_t control=4;control+4<best.controls.size();++control)
                    for(std::size_t axis=0;axis<3;++axis)
                        for(const auto direction:{-1.0,1.0}) {
                            auto candidate=best;
                            candidate.controls[control][axis]+=direction*step;
                            const auto measurement=measure(candidate,&source,24);
                            if(measurement.maxCurvature>original.maxCurvature*1.02) continue;
                            const auto candidateScore=score(measurement);
                            if(candidateScore<bestScore*(1.0-1e-10)) {
                                best=std::move(candidate);
                                bestMeasurement=measurement;
                                bestScore=candidateScore;
                                improved=true;
                            }
                        }
            }
        }
        const auto verified=measure(best,&source,256);
        if(verified.maxDeviation>limit*(1.0+1e-9))
            throw std::runtime_error("optimized quintic exceeds the verified 0.2P deviation bound");
        return best;
    }

    void writeComparison(const std::filesystem::path &base,const BSpline &cubic,const BSpline &quintic,const Composite &source,double feed){
        struct Row{double fraction=0,distance=0,cubic=0,quintic=0,deviation=0;};
        constexpr unsigned samples=4096;std::vector<Row> rows;rows.reserve(samples+1);
        const auto cubicSpans=cubic.controls.size()-cubic.degree,quinticSpans=quintic.controls.size()-quintic.degree;
        const SplineEvaluator cubicEvaluator(cubic),quinticEvaluator(quintic);
        const auto velocity=feed/60.0;
        for(unsigned i=0;i<=samples;++i){const auto f=double(i)/samples;const auto cg=cubicEvaluator.at(cubicSpans*f),qg=quinticEvaluator.at(quinticSpans*f);const auto cn=norm(sub(cg.derivative,mul(cg.tangent,dot(cg.tangent,cg.derivative))));const auto qn=norm(sub(qg.derivative,mul(qg.tangent,dot(qg.tangent,qg.derivative))));rows.push_back({f,source.total*f,cn,qn,norm(sub(qg.position,source.position(source.total*f)))});}
        auto path=base;path.replace_extension("csv");std::ofstream out(path,std::ios::trunc);out.precision(17);out<<"fraction,distance,cubic_normal_curvature_derivative,quintic_normal_curvature_derivative,cubic_jerk,quintic_jerk,quintic_deviation\n";for(const auto&r:rows)out<<r.fraction<<','<<r.distance<<','<<r.cubic<<','<<r.quintic<<','<<r.cubic*velocity*velocity*velocity<<','<<r.quintic*velocity*velocity*velocity<<','<<r.deviation<<'\n';if(!out)throw std::runtime_error("comparison write failed");
        path=base;path.replace_extension("svg");std::ofstream svg(path,std::ios::trunc);const auto maximumDerivative=std::ranges::max(rows,{},[](const Row&r){return std::max(r.cubic,r.quintic);});const auto maximumDeviation=std::max(0.00105,std::ranges::max(rows,{},&Row::deviation).deviation*1.05);const auto x=[](double f){return 90.0+1080.0*f;};const auto derivativeY=[&](double v){return 65.0+280.0*(1.0-std::log1p(v)/std::log1p(maximumDerivative.cubic>maximumDerivative.quintic?maximumDerivative.cubic:maximumDerivative.quintic));};const auto deviationY=[&](double v){return 430.0+200.0*(1.0-v/maximumDeviation);};svg<<"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1200\" height=\"700\" viewBox=\"0 0 1200 700\"><rect width=\"1200\" height=\"700\" fill=\"white\"/><style>text{font-family:Segoe UI,Arial,sans-serif;fill:#222}.axis{stroke:#222;stroke-width:1}.cubic{fill:none;stroke:#333;stroke-width:1.3}.quintic{fill:none;stroke:#1677ff;stroke-width:2}.deviation{fill:none;stroke:#18a558;stroke-width:1.7}.limit{stroke:#d62728;stroke-width:1.7;stroke-dasharray:8 5}</style><text x=\"90\" y=\"30\" font-size=\"21\" font-weight=\"600\">Production cubic vs optimized quintic at F"<<feed<<"</text><text x=\"90\" y=\"55\" font-size=\"15\">Normal curvature derivative (log scale)</text><line class=\"axis\" x1=\"90\" y1=\"65\" x2=\"90\" y2=\"345\"/><line class=\"axis\" x1=\"90\" y1=\"345\" x2=\"1170\" y2=\"345\"/><polyline class=\"cubic\" points=\"";for(const auto&r:rows)svg<<x(r.fraction)<<','<<derivativeY(r.cubic)<<' ';svg<<"\"/><polyline class=\"quintic\" points=\"";for(const auto&r:rows)svg<<x(r.fraction)<<','<<derivativeY(r.quintic)<<' ';const auto derivativeLimit=100.0/(velocity*velocity*velocity);svg<<"\"/><line class=\"limit\" x1=\"90\" y1=\""<<derivativeY(derivativeLimit)<<"\" x2=\"1170\" y2=\""<<derivativeY(derivativeLimit)<<"\"/><text x=\"90\" y=\"410\" font-size=\"15\">Quintic deviation from source primitives (inches)</text><line class=\"axis\" x1=\"90\" y1=\"430\" x2=\"90\" y2=\"630\"/><line class=\"axis\" x1=\"90\" y1=\"630\" x2=\"1170\" y2=\"630\"/><polyline class=\"deviation\" points=\"";for(const auto&r:rows)svg<<x(r.fraction)<<','<<deviationY(r.deviation)<<' ';svg<<"\"/><line class=\"limit\" x1=\"90\" y1=\""<<deviationY(0.001)<<"\" x2=\"1170\" y2=\""<<deviationY(0.001)<<"\"/><text x=\"600\" y=\"670\" text-anchor=\"middle\" font-size=\"15\">Fraction of primitive chain</text><line class=\"cubic\" x1=\"780\" y1=\"30\" x2=\"815\" y2=\"30\"/><text x=\"822\" y=\"35\" font-size=\"13\">production cubic</text><line class=\"quintic\" x1=\"950\" y1=\"30\" x2=\"985\" y2=\"30\"/><text x=\"992\" y=\"35\" font-size=\"13\">optimized quintic</text></svg>";if(!svg)throw std::runtime_error("comparison SVG write failed");
    }
}

int main(int argc,char **argv){try{if(argc!=5){std::cerr<<"usage: ngc_quintic_spline_analyzer <snapshot> <spline-id> <feed-per-minute> <output-base>\n";return 2;}const auto snapshot=readSnapshot(argv[1]);const auto id=std::stoull(argv[2]);const auto found=std::ranges::find(snapshot.splines,id,&CapturedSpline::id);if(found==snapshot.splines.end())throw std::runtime_error("spline not found");const auto source=compositeFor(snapshot,*found);BSpline cubic{3,found->controls,openKnots(found->controls.size(),3)};const auto rawQuintic=quinticFor(source,*found);const auto raw=measure(rawQuintic,&source);const auto quintic=optimizeQuinticPhysicalJerk(rawQuintic,source,0.005,0.2);const auto cm=measure(cubic),qm=measure(quintic,&source,256);writeComparison(argv[4],cubic,quintic,source,std::stod(argv[3]));std::cout<<"spline "<<id<<" cubic max_normal="<<cm.maxNormal<<" integral="<<cm.integralNormal<<" max_curvature="<<cm.maxCurvature<<"; raw_quintic max_normal="<<raw.maxNormal<<" integral="<<raw.integralNormal<<" deviation="<<raw.maxDeviation<<"; optimized_quintic max_normal="<<qm.maxNormal<<" integral="<<qm.integralNormal<<" max_curvature="<<qm.maxCurvature<<" deviation="<<qm.maxDeviation<<" length="<<qm.length<<'\n';return 0;}catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}}
