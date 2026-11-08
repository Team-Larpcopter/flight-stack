#pragma once
#include <navigation/route_guide.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <sstream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <array>

namespace larp_route_follower {
// Geometry only: bounded, resumable repair to an ordered CAD route. The caller
// supplies the same full-body collision predicate used for trajectory approval.
class MasterRoute {
 public:
  using Point=navigation::Point;using Polyline=navigation::Polyline;
  using Free=std::function<bool(const Point&,const Point&)>;
  explicit MasterRoute(Free free):free_(std::move(free)){}
  void load(const std::string& file) {
    std::ifstream in(file);if(!in)throw std::invalid_argument("cannot open master route: "+file);
    std::string line;master_.clear();gates_.clear();
    while(std::getline(in,line)) {
      if(line.rfind("# gates ",0)==0){std::istringstream s(line.substr(8));size_t i;while(s>>i)gates_.push_back(i);continue;}
      if(line.empty()||line[0]=='#')continue;
      std::istringstream s(line);Point p;
      if(!(s>>p.x()>>p.y()>>p.z())||!p.allFinite())throw std::invalid_argument("invalid master route row");
      master_.push_back(p);
    }
    if(master_.size()<2 || master_.size()>10000)throw std::invalid_argument("invalid master route size");
    for(size_t i=1;i<master_.size();++i)if((master_[i]-master_[i-1]).norm()> .151 || (master_[i]-master_[i-1]).norm()<1e-7)
      throw std::invalid_argument("master route must be sampled at <= 15 cm");
    if(gates_.empty()||gates_.back()!=master_.size()-1)gates_.push_back(master_.size()-1);
    size_t previous=0;for(auto i:gates_){if(i<=previous||i>=master_.size())throw std::invalid_argument("invalid passage order");previous=i;}
    remaining_.assign(master_.size(),0.);
    for(size_t i=master_.size()-1;i>0;--i)remaining_[i-1]=remaining_[i]+(master_[i]-master_[i-1]).norm();
  }
  bool activate(const Point& goal) {
    enabled_=master_.size()>1 && (master_.back()-goal).norm()<1e-4;
    if(!enabled_)return false;
    path_=master_;ids_.resize(path_.size());for(size_t i=0;i<ids_.size();++i)ids_[i]=i;
    cursor_=progress_=gate_=working_join_=0;cancel();expanded_=false;reason_="loaded CAD master route";return true;
  }
  bool enabled()const{return enabled_;}
  const Polyline& path()const{return path_;}
  const std::string& reason()const{return reason_;}
  size_t progress()const{return progress_;}
  size_t searches()const{return searches_;}
  size_t lastPops()const{return last_pops_;}
  size_t lastChecks()const{return checks_;}
  size_t nodeCount()const{return nodes_.size();}
  double lastMs()const{return last_ms_;}
  void setBounds(const Point& lo,const Point& hi){lower_=lo;upper_=hi;cancel();}

  Polyline nearby(const Point& start,double horizon,uint64_t revision) {
    const auto begin=Clock::now();deadline_=begin+std::chrono::milliseconds(2);
    checks_=last_pops_=0;
    struct Timer{MasterRoute* p;Clock::time_point t;~Timer(){p->last_ms_=std::chrono::duration<double,std::milli>(Clock::now()-t).count();}} timer{this,begin};
    if(!enabled_||!start.allFinite()||!revision)return {};
    if(revision!=revision_){edge_cache_.clear();bad_targets_.clear();revision_=revision;}
    // Advance only through the next local route segments, never nearest point
    // over the entire maze. A repair's interior does not advance master progress.
    // Rounded curves need not pass each original polyline vertex. Associate
    // only within the next metre of the active route, with a certified connector.
    // This cannot jump to an unrelated nearby corridor elsewhere in the maze.
    size_t best=cursor_;double best_distance=INFINITY,along=0.;Point projection=start;
    for(size_t i=cursor_;i+1<path_.size() && along<=1.;++i) {
      const Point d=path_[i+1]-path_[i];const double length=d.norm();
      if(length<1e-7)continue;
      const double t=std::clamp((start-path_[i]).dot(d)/d.squaredNorm(),0.,1.);
      const Point q=path_[i]+t*d;const double distance=(q-start).norm();
      if(distance<best_distance-1e-7){best_distance=distance;best=i;projection=q;}
      along+=length;
    }
    if(best_distance<.4 && available() && checked(start,projection)) {
      cursor_=best;progress_=std::max(progress_,ids_[cursor_]);
      if(cursor_+1<path_.size() && (start-path_[cursor_+1]).norm()<.03 && available() &&
         (cursor_+2==path_.size() || checked(start,path_[cursor_+2]))) {
        ++cursor_;progress_=std::max(progress_,ids_[cursor_]);
      }
    }
    while(gate_+1<gates_.size() && progress_>=gates_[gate_])++gate_;
    if(cursor_+1==path_.size() && checked(start,master_.back()))return {start,master_.back()};
    // Existing route prefix is useful while repair runs; the follower still
    // certifies the complete timed curve and its stopping endpoint.
    Polyline prefix{start};double length=0;size_t blocked=path_.size();
    for(size_t i=cursor_+1;i<path_.size() && length<horizon;++i) {
      const Point delta=path_[i]-prefix.back();const double d=delta.norm();if(d<1e-7)continue;
      const Point q=prefix.back()+delta*(std::min(d,horizon-length)/d);
      if(!available()){reason_="route validation slice";break;}
      if(!checked(prefix.back(),q)){blocked=i;break;}
      prefix.push_back(q);length+=std::min(d,horizon-length);
    }
    // Reserve a separate search slice so prefix validation cannot starve it.
    deadline_=begin+std::chrono::milliseconds(7);
    const bool prefix_clear=blocked==path_.size();
    const double needed=std::min(horizon,(start-master_.back()).norm());
    if(blocked==path_.size() && length+.02>=needed) {
      reason_="following cached CAD route; periodic forward repair";
    }
    const size_t first=std::max({progress_+1,working_join_,blocked<ids_.size()?ids_[blocked]:progress_+1});
    if((pending_ && (start-search_start_).norm()>.2) ||
       ((pending_ || !solution_.empty()) && search_first_<first))cancel();
    if(!pending_ && solution_.empty()) {
      // Forward-only candidate indices; never discard the original master tail.
      beginSearch(start,first);
    }
    if(pending_ && available())advance();
    if(!solution_.empty()) {
      // Revalidate each proposed repair edge against the latest revision over
      // slices. Map changes restart validation, not the entire A* search.
      if(solution_revision_!=revision_){solution_revision_=revision_;validate_at_=1;tail_at_=solution_join_;tail_distance_=0;}
      while(validate_at_<solution_.size() && available()) {
        if(!checked(solution_[validate_at_-1],solution_[validate_at_])) {
          cancel();reason_="repair changed during search; retry";return safePrefix(prefix);
        }
        ++validate_at_;
      }
      while(validate_at_==solution_.size() && tail_at_+1<master_.size() && tail_distance_<horizon && available()) {
        if(!checked(master_[tail_at_],master_[tail_at_+1])) {
          bad_targets_.insert(solution_join_);cancel();reason_="forward tail blocked; continue periodic search";
          return prefix_clear?simplify(prefix):safePrefix(prefix);
        }
        tail_distance_+=(master_[tail_at_+1]-master_[tail_at_]).norm();++tail_at_;
      }
      if(validate_at_==solution_.size() && (tail_at_+1==master_.size() || tail_distance_>=horizon)) {
        double old_cost=(start-path_[cursor_]).norm(),new_cost=remaining_[solution_join_];
        for(size_t i=cursor_+1;i<path_.size();++i)old_cost+=(path_[i]-path_[i-1]).norm();
        for(size_t i=1;i<solution_.size();++i)new_cost+=(solution_[i]-solution_[i-1]).norm();
        // Retain the working route when the new connection offers no improvement.
        if(blocked==path_.size() && length+.02>=needed && new_cost>=old_cost-.02) {
          cancel();reason_="periodic search retained working route";return simplify(prefix);
        }
        working_join_=solution_join_;
        path_=solution_;ids_.assign(path_.size(),progress_);ids_.back()=solution_join_;
        for(size_t i=solution_join_+1;i<master_.size();++i){path_.push_back(master_[i]);ids_.push_back(i);}
        cursor_=0;cancel();expanded_=false;reason_="local repair joined forward CAD route";
        // Next invocation obtains and certifies the new prefix. Do not recurse
        // and accidentally spend a second work slice in this invocation.
      }
    }
    return prefix_clear?simplify(prefix):safePrefix(prefix);
  }
 private:
  using Clock=std::chrono::steady_clock;
  struct Key {int x,y,z;bool operator==(const Key& b)const{return x==b.x&&y==b.y&&z==b.z;}};
  struct Hash {size_t operator()(const Key& k)const{return size_t(k.x)*73856093u ^ size_t(k.y)*19349663u ^ size_t(k.z)*83492791u;}};
  struct Node {double g=INFINITY;Key parent{};bool has_parent=false,closed=false;};
  struct Entry {double f,g;Key key;bool operator<(const Entry& b)const{return f>b.f;}};
  struct Target {Key key;size_t index;double terminal;};
  struct CachedEdge {Point a,b;bool free;};
  bool available()const{return checks_<64 && last_pops_<256 && Clock::now()<deadline_;}
  bool checked(const Point& a,const Point& b) {
    // Cache only within one map revision; bounded memory, no map copies.
    size_t key=0;for(int j=0;j<3;++j){key^=std::hash<double>{}(a[j])+0x9e3779b9+(key<<6)+(key>>2);key^=std::hash<double>{}(b[j])+0x9e3779b9+(key<<6)+(key>>2);}
    auto it=edge_cache_.find(key);
    if(it!=edge_cache_.end() && it->second.a==a && it->second.b==b)return it->second.free;
    ++checks_;const bool ok=(a.array()>=lower_.array()).all()&&(a.array()<=upper_.array()).all()&&
      (b.array()>=lower_.array()).all()&&(b.array()<=upper_.array()).all()&&free_(a,b);
    if(edge_cache_.size()>=32768)edge_cache_.clear();
    edge_cache_.insert_or_assign(key,CachedEdge{a,b,ok});return ok;
  }
  static Polyline safePrefix(Polyline p) {
    // Leave a 15 cm margin to a blocked edge for the stopping endpoint.
    double cut=.15;
    while(p.size()>1 && cut>0){const double d=(p.back()-p[p.size()-2]).norm();if(d<=cut){cut-=d;p.pop_back();}
      else{p.back()-=cut*(p.back()-p[p.size()-2]).normalized();cut=0;}}
    return p.size()>1?simplify(p):Polyline{};
  }
  static Polyline simplify(const Polyline& p) {
    Polyline out;
    for(const auto& q:p){if(!out.empty()&&(q-out.back()).norm()<1e-7)continue;
      if(out.size()>1){const Point a=out.back()-out[out.size()-2],b=q-out.back();if(a.cross(b).norm()<1e-8 && a.dot(b)>0)out.pop_back();}out.push_back(q);}
    return out;
  }
  Point pos(Key k)const{return search_start_+.1*Point(k.x,k.y,k.z);}
  Key key(const Point& p)const{const auto k=((p-search_start_)/.1).array().round().cast<int>();return {k.x(),k.y(),k.z()};}
  static double diagonal(Key a,Key b){std::array<int,3>d{std::abs(a.x-b.x),std::abs(a.y-b.y),std::abs(a.z-b.z)};std::sort(d.begin(),d.end());return .1*(std::sqrt(3.)*d[0]+std::sqrt(2.)*(d[1]-d[0])+d[2]-d[1]);}
  double heuristic(Key k)const{double h=INFINITY;for(const auto& t:targets_)h=std::min(h,diagonal(k,t.key)+t.terminal);return h;}
  void cancel(){pending_=false;open_={};nodes_.clear();targets_.clear();solution_.clear();expanding_=false;best_=INFINITY;}
  void beginSearch(const Point& start,size_t first) {
    cancel();search_first_=first;search_start_=start;search_lo_=start;search_hi_=start;
    const size_t last=gates_[gate_];
    size_t previous=first;
    for(size_t i=first;i<=last && i<master_.size() && targets_.size()<24;++i) {
      if(remaining_[first]-remaining_[i]>(expanded_?6.:3.))break;
      if(i!=first && i!=last && remaining_[previous]-remaining_[i]<.29)continue;
      if(bad_targets_.count(i))continue;
      previous=i;
      Key k=key(master_[i]);targets_.push_back({k,i,remaining_[i]+(pos(k)-master_[i]).norm()});
      search_lo_=search_lo_.cwiseMin(master_[i]);search_hi_=search_hi_.cwiseMax(master_[i]);
    }
    if(targets_.empty()){reason_="no forward rejoin before required passage";retry_after_=Clock::now()+std::chrono::milliseconds(250);return;}
    const double margin=expanded_?1.2:.6;
    search_lo_=(search_lo_-Point::Constant(margin)).cwiseMax(lower_);search_hi_=(search_hi_+Point::Constant(margin)).cwiseMin(upper_);
    nodes_[{0,0,0}].g=0;open_.push({heuristic({0,0,0}),0,{0,0,0}});pending_=true;++searches_;reason_="repairing to forward CAD tail";
  }
  void advance() {
    while(available()) {
      if(!expanding_) {
        if(open_.empty() || open_.top().f>=best_) {
          if(std::isfinite(best_)) {
            Key at=best_key_;Polyline reverse{master_[solution_join_],pos(at)};
            while(nodes_.at(at).has_parent){at=nodes_.at(at).parent;reverse.push_back(pos(at));}
            solution_=simplify(Polyline(reverse.rbegin(),reverse.rend()));
            validate_at_=1;tail_at_=solution_join_;tail_distance_=0;solution_revision_=revision_;pending_=false;reason_="validating local repair";return;
          }
          cancel();expanded_=!expanded_;retry_after_=Clock::now()+std::chrono::milliseconds(250);reason_="local repair blocked; hold/scan";return;
        }
        auto entry=open_.top();open_.pop();++last_pops_;
        auto& n=nodes_.at(entry.key);if(n.closed||entry.g!=n.g)continue;
        current_=entry.key;current_g_=n.g;neighbor_=0;target_at_=0;expanding_=true;
      }
      while(target_at_<targets_.size() && available()) {
        const auto& t=targets_[target_at_];if(!(current_==t.key)){++target_at_;continue;}
        if(checks_+2>64)return;
        ++target_at_;
        // Also check the forward half metre: never reconnect to an isolated
        // route vertex whose continuation is blocked.
        bool ok=checked(pos(current_),master_[t.index]);
        size_t onward=t.index;
        while(onward+1<master_.size() && remaining_[t.index]-remaining_[onward+1]<=.3)++onward;
        if(ok && onward>t.index)ok=checked(master_[t.index],master_[onward]);
        if(ok && current_g_+t.terminal<best_){best_=current_g_+t.terminal;best_key_=current_;solution_join_=t.index;}
      }
      while(neighbor_<27 && available()) {
        const int i=neighbor_++;const Key next{current_.x+i/9-1,current_.y+(i/3)%3-1,current_.z+i%3-1};
        if(next==current_)continue;const Point p=pos(next);
        if((p.array()<search_lo_.array()).any()||(p.array()>search_hi_.array()).any())continue;
        const double g=current_g_+diagonal(current_,next);
        const auto it=nodes_.find(next);if(it!=nodes_.end()&&(it->second.closed||it->second.g<=g))continue;
        if(!checked(pos(current_),p))continue;
        if(nodes_.size()>=40000){cancel();reason_="local repair node budget exhausted";retry_after_=Clock::now()+std::chrono::milliseconds(250);return;}
        auto& n=nodes_[next];n.g=g;n.parent=current_;n.has_parent=true;open_.push({g+heuristic(next),g,next});
      }
      if(neighbor_==27){nodes_.at(current_).closed=true;expanding_=false;}
    }
  }
  Free free_;Polyline master_,path_,solution_;std::vector<size_t> gates_,ids_;std::vector<double> remaining_;
  std::unordered_set<size_t> bad_targets_;
  size_t tail_at_=0;double tail_distance_=0;
  size_t search_first_=0,working_join_=0;
  size_t cursor_=0,progress_=0,gate_=0,searches_=0,last_pops_=0,checks_=0,validate_at_=0,solution_join_=0;
  uint64_t revision_=0,solution_revision_=0;
  bool enabled_=false,pending_=false,expanded_=false,expanding_=false;
  Point lower_=Point::Constant(-INFINITY),upper_=Point::Constant(INFINITY),search_start_,search_lo_,search_hi_;
  std::unordered_map<Key,Node,Hash> nodes_;std::priority_queue<Entry> open_;std::vector<Target> targets_;
  std::unordered_map<size_t,CachedEdge> edge_cache_;
  Key current_{},best_key_{};int neighbor_=0;size_t target_at_=0;double current_g_=0,best_=INFINITY,last_ms_=0;
  Clock::time_point deadline_,retry_after_{};std::string reason_;
};
}
