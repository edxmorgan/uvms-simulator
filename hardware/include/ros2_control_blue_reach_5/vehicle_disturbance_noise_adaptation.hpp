#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>
#include <dlfcn.h>
#include <string>
#include <Eigen/Eigenvalues>
#include <casadi/casadi.hpp>

namespace ros2_control_blue_reach_5 {

// Causal marginal disturbance-increment covariance matching, NOT full-state EM.
// This worker has no truth, geometry, control or actuator access. The caller must
// gate observational availability and clear on sensor/model/reset changes. Full
// non-diagonal configured Q is not supported by this adaptation model at configuration time. Learned Q retains all6x6 entries.
class VehicleDisturbanceNoiseAdaptation {
 public:
  using Density = std::array<double, 6>;
  using Covariance = std::array<double,36>;
  struct Snapshot {
    // Dense matrices use CasADi's column-major order. Prior/A are the transition
    // FROM the preceding sample TO this sample; x/P are this sample's posterior.
    std::array<double, 18> filtered{}, prior{};
    std::array<double, 324> filtered_covariance{}, prior_covariance{}, transition{};
    Covariance applied_covariance{};
    double dt{0.};
    uint64_t step{0};
  };
  using Step = std::function<void(const Snapshot&, const Snapshot&,
      const std::array<double,18>&, const std::array<double,324>&,
      std::array<double,18>&, std::array<double,324>&, Covariance&)>;
  struct Result {
    Density density{};
    Covariance covariance{};
    uint64_t generation{0}, endpoint_step{0};
    size_t transitions{0};
    double window_seconds{0.}, fit_seconds{0.};
  };

  explicit VehicleDisturbanceNoiseAdaptation(Step step, Density configured_density)
      : smoother_(std::move(step)), defaults_(configured_density) {
    if (!smoother_) throw std::invalid_argument("missing generated RTS moment helper");
    for (double v : defaults_)
      if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("invalid configured disturbance density");
    worker_ = std::thread([this] { run(); });
  }
  explicit VehicleDisturbanceNoiseAdaptation(const std::string& library, Density defaults)
      : VehicleDisturbanceNoiseAdaptation(raw_helper(library), defaults) {}
  ~VehicleDisturbanceNoiseAdaptation() {
    {std::lock_guard<std::mutex> lock(queue_mutex_); stopping_.store(true);}
    available_.notify_one();
    if (worker_.joinable()) worker_.join();
  }
  VehicleDisturbanceNoiseAdaptation(const VehicleDisturbanceNoiseAdaptation&) = delete;
  VehicleDisturbanceNoiseAdaptation& operator=(const VehicleDisturbanceNoiseAdaptation&) = delete;

  // Never waits for the worker. A lost snapshot breaks the consecutive-step
  // contract: the next received step clears history before collecting a new fit.
  bool submit(const Snapshot& snapshot) {
    std::unique_lock<std::mutex> lock(queue_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || queued_ == queue_capacity) { ++dropped_; return false; }
    queue_[(head_ + queued_) % queue_capacity] = {snapshot, generation_.load()};
    ++queued_; lock.unlock(); available_.notify_one(); return true;
  }
  // Atomically invalidates queued/in-flight fits without waiting or changing Q.
  void clear() { ++generation_; available_.notify_one(); }
  uint64_t generation() const { return generation_.load(); }
  uint64_t dropped() const { return dropped_.load(); }
  uint64_t failed() const { return failed_.load(); }
  std::optional<Result> poll() {
    std::unique_lock<std::mutex> lock(result_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !result_) return std::nullopt;
    auto result = result_; result_.reset();
    if (result->generation != generation_.load()) return std::nullopt;
    return result;
  }

 private:
  static constexpr size_t queue_capacity = 128, history_capacity = 2048;
  static constexpr double window_seconds = 2., update_seconds = .5;
  struct Queued { Snapshot snapshot; uint64_t generation{0}; };
  Step smoother_;
  Density defaults_;
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  std::atomic<uint64_t> generation_{0}, dropped_{0}, failed_{0};
  std::mutex queue_mutex_, result_mutex_;
  std::condition_variable available_;
  std::array<Queued, queue_capacity> queue_{};
  size_t head_{0}, queued_{0};
  std::optional<Result> result_;

  // Generated plain C ABI only: CasADi runtime sparsity caches in the installed
  // build are not thread-safe. Each worker owns all pointer/numeric workspace.
  struct RawHelper {
    using Integer=long long;
    using Evaluate=int(*)(const double**,double**,Integer*,double*,int);
    void* library{nullptr}; Evaluate evaluate{nullptr};
    void(*release)(int){nullptr}; void(*decref)(){nullptr}; int memory{-1};
    std::vector<const double*> args; std::vector<double*> results;
    std::vector<Integer> integers; std::vector<double> work;
    template<class T> T symbol(const char* suffix) {
      const std::string name=std::string("disturbance_ekf_smoothing_full_moment")+suffix;
      auto ptr=dlsym(library,name.c_str());
      if(!ptr) throw std::runtime_error("missing native RTS symbol: "+name);
      return reinterpret_cast<T>(ptr);
    }
    explicit RawHelper(const std::string& path) {
      library=dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL);
      if(!library) throw std::runtime_error(std::string("cannot load native RTS helper: ")+dlerror());
      try {
        evaluate=symbol<Evaluate>("");
        auto size=symbol<int(*)(Integer*,Integer*,Integer*,Integer*)>("_work");
        Integer na=0,nr=0,ni=0,nw=0;
        if(size(&na,&nr,&ni,&nw)!=0 || na<7 || nr<3 || ni<0 || nw<0)
          throw std::runtime_error("invalid native RTS workspace");
        auto inputs=symbol<const Integer*(*)(Integer)>("_sparsity_in");
        auto outputs=symbol<const Integer*(*)(Integer)>("_sparsity_out");
        const int rows[]={18,18,18,18,18,18,18}, cols[]={1,18,1,18,18,1,18};
        for(int i=0;i<7;++i) {auto v=inputs(i); if(!v || v[0]!=rows[i] || v[1]!=cols[i] || v[2]!=1)
          throw std::runtime_error("native RTS inputs must be dense");}
        for(int i=0;i<3;++i) {auto v=outputs(i); if(!v || v[0]!=(i==2?6:18) || v[1]!=(i==1?18:(i==2?6:1)) || v[2]!=1)
          throw std::runtime_error("native RTS outputs must be dense");}
        args.resize(na,nullptr); results.resize(nr,nullptr); integers.resize(ni); work.resize(nw);
        symbol<void(*)()>("_incref")(); decref=symbol<void(*)()>("_decref");
        release=symbol<void(*)(int)>("_release"); memory=symbol<int(*)()>("_checkout")();
        if(memory<0) throw std::runtime_error("native RTS checkout failed");
      } catch(...) {close(); throw;}
    }
    void close() {
      if(memory>=0 && release) release(memory);
      if(decref) decref();
      if(library) dlclose(library);
      library=nullptr; memory=-1; decref=nullptr;
    }
    ~RawHelper(){close();}
    void call(const Snapshot& a,const Snapshot& b,const std::array<double,18>& next_x,
              const std::array<double,324>& next_P,std::array<double,18>& x,
              std::array<double,324>& P,Covariance& moments) {
      std::fill(args.begin(),args.end(),nullptr); std::fill(results.begin(),results.end(),nullptr);
      args[0]=a.filtered.data(); args[1]=a.filtered_covariance.data();
      args[2]=b.prior.data(); args[3]=b.prior_covariance.data(); args[4]=b.transition.data();
      args[5]=next_x.data(); args[6]=next_P.data();
      results[0]=x.data(); results[1]=P.data(); results[2]=moments.data();
      if(evaluate(args.data(),results.data(),integers.data(),work.data(),memory)!=0)
        throw std::runtime_error("native RTS evaluation failed");
    }
  };
  static Step raw_helper(const std::string& path) {
    auto helper=std::make_shared<RawHelper>(path);
    return [helper](const Snapshot& a,const Snapshot& b,const std::array<double,18>& xs,
                   const std::array<double,324>& Ps,std::array<double,18>& x,
                   std::array<double,324>& P,Covariance& moments) {helper->call(a,b,xs,Ps,x,P,moments);};
  }
  static bool finite(const Snapshot& s) {
    if (!std::isfinite(s.dt) || s.dt <= 0. || s.dt >= window_seconds) return false;
    for (double v:s.applied_covariance) if (!std::isfinite(v)) return false;
    for(size_t i=0;i<6;++i) if(s.applied_covariance[7*i]<=0.) return false;
    for (const auto* a : {&s.filtered_covariance, &s.prior_covariance, &s.transition})
      for (double v:*a) if (!std::isfinite(v)) return false;
    for (const auto* a : {&s.filtered, &s.prior})
      for (double v:*a) if (!std::isfinite(v)) return false;
    return true;
  }
  Result fit(const std::deque<Snapshot>& history, uint64_t generation) {
    const auto started = std::chrono::steady_clock::now();
    auto xs=history.back().filtered; auto Ps=history.back().filtered_covariance;
    Covariance sum{};
    double duration=0., minimum_dt=window_seconds;
    for (size_t k=history.size()-1; k>0; --k) {
      if (stopping_.load() || generation != generation_.load())
        throw std::runtime_error("superseded adaptation window");
      const auto& a=history[k-1]; const auto& b=history[k];
      // Scientific smoother/moment equations live in the generated source.
      std::array<double,18> previous_x{}; std::array<double,324> previous_P{}; Covariance moments{};
      smoother_(a,b,xs,Ps,previous_x,previous_P,moments);
      for(double v:previous_x) if(!std::isfinite(v)) throw std::runtime_error("nonfinite RTS state");
      for(double v:previous_P) if(!std::isfinite(v)) throw std::runtime_error("nonfinite RTS covariance");
      for(double v:moments) if(!std::isfinite(v)) throw std::runtime_error("nonfinite RTS moments");
      xs=previous_x; Ps=previous_P; duration+=b.dt; minimum_dt=std::min(minimum_dt,b.dt);
      for(size_t i=0;i<36;++i) sum[i]+=moments[i]/b.dt;
    }
    Result result; result.generation=generation; result.endpoint_step=history.back().step;
    result.transitions=history.size()-1; result.window_seconds=duration;
    using Matrix=Eigen::Matrix<double,6,6,Eigen::ColMajor>;
    const Eigen::Map<const Matrix> raw(sum.data());
    const Matrix symmetric=(raw+raw.transpose())*.5;
    Eigen::SelfAdjointEigenSolver<Matrix> decomposition(symmetric);
    // Validator tolerance1e-10 on the unnormalized summed moments; dividing
    // each interval by dt scales that numerical tolerance by1/minimum_dt.
    if(decomposition.info()!=Eigen::Success || !decomposition.eigenvalues().allFinite() ||
       decomposition.eigenvalues().minCoeff() < -1e-10/minimum_dt)
      throw std::runtime_error("materially indefinite disturbance moments");
    const Matrix positive=decomposition.eigenvectors()*
      decomposition.eigenvalues().cwiseMax(0.).asDiagonal()*decomposition.eigenvectors().transpose();
    const double n=double(result.transitions);
    for(size_t j=0;j<6;++j) for(size_t i=0;i<6;++i) {
      const size_t k=i+6*j;
      const double prior=(i==j?defaults_[i]:0.);
      const double target=(positive(i,j)+.1*n*prior)/(1.1*n);
      result.covariance[k]=.5*history.back().applied_covariance[k]+.5*target;
      if(!std::isfinite(result.covariance[k])) throw std::runtime_error("nonfinite matched disturbance covariance");
    }
    for(size_t i=0;i<6;++i) {
      result.density[i]=result.covariance[7*i];
      if(result.density[i]<=0.) throw std::runtime_error("invalid matched disturbance density");
    }
    result.fit_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    return result;
  }
  void run() {
    std::deque<Snapshot> history;
    uint64_t active_generation=generation_.load();
    double duration=0., elapsed=0., last_fit=0.;
    auto reset=[&] { history.clear(); duration=elapsed=last_fit=0.; };
    while(!stopping_.load()) {
      Queued item;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        available_.wait(lock,[&] {return stopping_.load() || queued_!=0;});
        if(stopping_.load()) return;
        item=queue_[head_]; head_=(head_+1)%queue_capacity; --queued_;
      }
      if(item.generation!=generation_.load()) continue;
      if(item.generation!=active_generation) {reset(); active_generation=item.generation;}
      const auto& s=item.snapshot;
      if(!finite(s)) {reset(); ++failed_; continue;}
      if(!history.empty() && s.step!=history.back().step+1) reset();
      history.push_back(s); duration+=s.dt; elapsed+=s.dt;
      while(history.size()>2 && duration-history.front().dt>=window_seconds-1e-10) {
        duration-=history.front().dt; history.pop_front();
      }
      if(history.size()>history_capacity) {reset(); ++failed_; continue;}
      if(duration<window_seconds-1e-10 || history.size()<2 ||
          elapsed<=window_seconds+1e-10 || elapsed-last_fit<update_seconds-1e-10) continue;
      last_fit=elapsed;
      try {
        auto fitted=fit(history,active_generation);
        if(fitted.generation==generation_.load()) {
          std::lock_guard<std::mutex> lock(result_mutex_); result_=fitted;
        }
      } catch(const std::exception&) {
        if(active_generation==generation_.load() && !stopping_.load()) ++failed_;
      }
    }
  }
};

// Shared lifecycle/telemetry binding for simulation and hardware. All filtering
// and sensor-availability decisions remain with the caller. No actuator access.
class VehicleDisturbanceNoiseBinding {
 public:
  using Density = VehicleDisturbanceNoiseAdaptation::Density;
  Density active_density{};
  // enabled, applied fit count, dropped snapshots, failed fits, worker fit seconds,
  // endpoint step. These stable addresses may be exported as read-only GPIOs.
  std::array<double,6> diagnostics{};

  void configure(Density configured, const std::string& library) {
    stop();
    for(double q:configured)
      if(!std::isfinite(q) || q<0.) throw std::invalid_argument("invalid configured disturbance density");
    defaults_=active_density=configured;
    active_covariance_.fill(0.);
    for(size_t i=0;i<6;++i) active_covariance_[7*i]=configured[i];
    diagnostics.fill(0.); local_failures_=0.;
    if(!library.empty()) {
      worker_=std::make_unique<VehicleDisturbanceNoiseAdaptation>(library,defaults_);
      diagnostics[0]=1.;
    }
  }
  // Lifecycle-only: joins the worker; never call stop/configure in a read cycle.
  void stop() {
    worker_.reset(); ready_=false; pending_reset_=false; diagnostics[0]=0.;
  }
  void reset(casadi::DM& Q) {
    ready_=false; pending_reset_=false;
    if(!worker_) return; // preserve disabled full covariance overrides exactly
    worker_->clear(); active_density=defaults_;
    active_covariance_.fill(0.);
    for(size_t i=0;i<6;++i) active_covariance_[7*i]=defaults_[i];
    write_density(Q);
    diagnostics[1]=diagnostics[4]=diagnostics[5]=0.;
    refresh_counts();
  }
  void before_prediction(casadi::DM& Q, bool available) {
    if(!worker_) return;
    if(pending_reset_) reset(Q);
    if(!available) {
      if(ready_) reset(Q);
      refresh_counts(); return;
    }
    ready_=true;
    if(auto result=worker_->poll()) {
      active_density=result->density; active_covariance_=result->covariance; write_density(Q);
      diagnostics[1]+=1.; diagnostics[4]=result->fit_seconds;
      diagnostics[5]=double(result->endpoint_step);
    }
    refresh_counts();
  }
  void after_correction(const std::vector<casadi::DM>& prediction,
                        const std::vector<casadi::DM>& corrected,
                        double dt, uint64_t step) {
    if(!worker_ || !ready_) return;
    VehicleDisturbanceNoiseAdaptation::Snapshot snapshot;
    snapshot.dt=dt; snapshot.step=step; snapshot.applied_covariance=active_covariance_;
    try {
      if(prediction.size()<3 || corrected.size()<2)
        throw std::invalid_argument("incomplete vehicle EKF snapshot");
      copy_dense(corrected[0],snapshot.filtered);
      copy_dense(corrected[1],snapshot.filtered_covariance);
      copy_dense(prediction[0],snapshot.prior);
      copy_dense(prediction[1],snapshot.prior_covariance);
      copy_dense(prediction[2],snapshot.transition);
      worker_->submit(snapshot);
    } catch(const std::exception&) {
      // Bad telemetry cannot supply a consecutive smoothing window. Do not
      // replace operative Q from an old fit after this failed snapshot.
      worker_->clear(); ready_=false; pending_reset_=true; ++local_failures_;
    }
    refresh_counts();
  }
 private:
  std::unique_ptr<VehicleDisturbanceNoiseAdaptation> worker_;
  Density defaults_{};
  VehicleDisturbanceNoiseAdaptation::Covariance active_covariance_{};
  bool ready_{false}, pending_reset_{false};
  double local_failures_{0.};
  void write_density(casadi::DM& Q) {
    if(Q.size1()!=18 || Q.size2()!=18) throw std::invalid_argument("vehicle Q must be18x18");
    for(size_t j=0;j<6;++j) for(size_t i=0;i<6;++i) Q(12+i,12+j)=active_covariance_[i+6*j];
  }
  void refresh_counts() {
    if(worker_) {diagnostics[2]=double(worker_->dropped()); diagnostics[3]=double(worker_->failed())+local_failures_;}
  }
  template<size_t N> static void copy_dense(const casadi::DM& source,std::array<double,N>& target) {
    if(source.numel()!=N) throw std::invalid_argument("invalid vehicle EKF snapshot shape");
    const auto values=casadi::DM::densify(source).nonzeros();
    std::copy(values.begin(),values.end(),target.begin());
  }
};
} // namespace ros2_control_blue_reach_5
