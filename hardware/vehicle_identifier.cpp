#include "ros2_control_blue_reach_5/vehicle_identifier.hpp"
#include "fusion.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace uvms::identification {
using namespace mosek::fusion;
using namespace monty;
using Clock = std::chrono::steady_clock;
namespace {
double elapsed(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
auto array(const Eigen::VectorXd & v) {
  return new_array_ptr<double>(std::vector<double>(v.data(), v.data()+v.size()));
}
double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return (v[(v.size()-1)/2]+v[v.size()/2])*.5;
}
Vector6 rms(const Eigen::VectorXd & residual) {
  Vector6 sum = Vector6::Zero();
  for (Eigen::Index i=0; i<residual.size(); ++i) sum[i%6] += residual[i]*residual[i];
  return (sum/(residual.size()/6)).cwiseSqrt();
}
}
struct Identifier::Impl {
  double lower, upper, budget;
  Model::t model;
  Parameter::t scale, inverse_scale, factor, target;
  Variable::t coordinates;
  Eigen::MatrixXd basis;
  Impl(double lo, double hi, double seconds): lower(lo), upper(hi), budget(seconds) {
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo<=0 || hi<lo ||
        !std::isfinite(seconds) || seconds<=0) throw std::invalid_argument("Invalid physical bounds/budget");
    model = new Model("vehicle_identification");
    try {
      scale=model->parameter("scale",25);
      inverse_scale=model->parameter("inverse_scale",27);
      factor=model->parameter("factor",new_array_ptr<int,1>({27,27}));
      target=model->parameter("target",27);
      coordinates=model->variable("coordinates",25,Domain::unbounded());
      // Eliminate only the two exact symmetry equalities. A shared variable
      // prevents solver feasibility tolerances from making M asymmetric.
      // This parameterizes precisely the original feasible set, without a prior.
      std::vector<double> embedding(27*25,0.);
      for(int i=0,j=0;i<27;++i){
        if(i==5 || i==6)continue;
        embedding[i*25+j]=1.;
        if(i==4)embedding[5*25+j]=1.;
        if(i==3)embedding[6*25+j]=1.;
        ++j;
      }
      auto theta=Expr::mul(Matrix::dense(27,25,new_array_ptr<double>(embedding)),Expr::mulElm(scale,coordinates));
      auto t=model->variable("objective",1,Domain::unbounded());
      model->constraint(Expr::vstack(t,Expr::vstack(.5,Expr::sub(
        Expr::mul(factor,Expr::mulElm(inverse_scale,theta)),target))),Domain::inRotatedQCone());
      model->objective(ObjectiveSense::Minimize,t);
      // Exact original 6x6 PSD constraint, including symmetry (not a diagonal approximation).
      auto psd=model->variable("inertia_minus_floor",Domain::inPSDCone(6));
      const int map[6][6]={{0,-1,-1,-1,3,-1},{-1,1,-1,4,-1,-1},
        {-1,-1,2,-1,-1,-1},{-1,5,-1,7,-1,-1}, {6,-1,-1,-1,8,-1}, {-1,-1,-1,-1,-1,9}};
      for (int i=0;i<6;++i) for (int j=0;j<6;++j) {
        auto entry=psd->index(i,j);
        if (map[i][j]>=0)
          model->constraint(Expr::sub(theta->index(map[i][j]),entry),Domain::equalsTo(i==j?1e-6:0.));
        else model->constraint(entry,Domain::equalsTo(0.));
      }
      model->constraint(theta->slice(15,27),Domain::lessThan(0.));
      model->constraint(theta->index(10),Domain::inRange(lo,hi));
      model->setSolverParam("numThreads",1);
      model->setSolverParam("intpntCoTolRelGap",1e-10);
      model->setSolverParam("intpntCoTolPfeas",1e-10);
      model->setSolverParam("intpntCoTolDfeas",1e-10);
      Eigen::MatrixXd e=Eigen::MatrixXd::Zero(lo==hi?3:2,27);
      e(0,3)=e(1,4)=1; e(0,6)=e(1,5)=-1;
      if(lo==hi)e(2,10)=1;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(e,Eigen::ComputeFullV);
      basis=svd.matrixV().rightCols(27-e.rows());
    } catch (...) { model->dispose(); throw; }
  }
  ~Impl(){model->dispose();}
  Fit solve(const Eigen::MatrixXd & a,const Eigen::VectorXd & b,double remaining) {
    Parameters s;
    for(int j=0;j<27;++j){double n=a.col(j).norm();s[j]=std::clamp(n>0?1/n:1.,1e-3,1e3);}
    Eigen::MatrixXd scaled=a*s.asDiagonal();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(scaled);
    Eigen::MatrixXd r=qr.matrixQR().topRows(27).triangularView<Eigen::Upper>();
    Eigen::VectorXd y=(qr.householderQ().adjoint()*b).head(27);
    std::vector<double> row_major(729);
    for(int i=0;i<27;++i)for(int j=0;j<27;++j)row_major[i*27+j]=r(i,j);
    Eigen::VectorXd independent_scale(25);
    for(int i=0,j=0;i<27;++i)if(i!=5 && i!=6)independent_scale[j++]=s[i];
    scale->setValue(array(independent_scale));inverse_scale->setValue(array(s.cwiseInverse()));
    factor->setValue(new_array_ptr<double>(row_major)); target->setValue(array(y));
    model->setSolverParam("optimizerMaxTime",remaining);
    model->solve();
    if(model->getPrimalSolutionStatus()!=SolutionStatus::Optimal)
      throw std::runtime_error("MOSEK did not return an optimal solution");
    auto level=coordinates->level(); Fit fit;
    for(int i=0,j=0;i<27;++i)if(i!=5 && i!=6)fit.parameters[i]=(*level)[j++]*s[i];
    fit.parameters[5]=fit.parameters[4];fit.parameters[6]=fit.parameters[3];
    fit.solver_seconds=model->getSolverDoubleInfo("optimizerTime");
    const auto & p=fit.parameters;
    auto m=inertia_matrix(p);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig((m+m.transpose())*.5);
    if(!p.allFinite() || !std::isfinite(fit.solver_seconds) || fit.solver_seconds>remaining ||
       (m-m.transpose()).cwiseAbs().maxCoeff()>1e-6 || eig.info()!=Eigen::Success ||
       eig.eigenvalues().minCoeff()<0 || p.tail<12>().maxCoeff()>1e-6 || p[10]<lower-1e-6 || p[10]>upper+1e-6)
    {
      std::ostringstream reason;
      reason << "Rejected MOSEK candidate: finite=" << p.allFinite()
             << " symmetry_error=" << (m-m.transpose()).cwiseAbs().maxCoeff()
             << " min_inertia_eigenvalue=" << eig.eigenvalues().minCoeff()
             << " max_drag=" << p.tail<12>().maxCoeff() << " weight=" << p[10]
             << " solver_seconds=" << fit.solver_seconds << " remaining=" << remaining;
      throw std::runtime_error(reason.str());
    }
    return fit;
  }
};
Identifier::Identifier(double lo,double hi,double budget):impl_(new Impl(lo,hi,budget)){}
void Identifier::preflight() {
  Parameters p=Parameters::Zero();
  p.head<3>().setConstant(15.);p.segment<3>(7).setConstant(2.);
  p[10]=impl_->lower;p[11]=impl_->lower;p.tail<12>().setConstant(-2.);
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(54,27);
  a.topRows(27).setIdentity();a.bottomRows(27).setIdentity();
  // This asynchronous startup probe includes first-use license/runtime setup,
  // not a live measurement fit. Do not permanently disable identification
  // because that initialization exceeds the live-fit deadline. Keep a separate
  // optimizer limit here; fit() still enforces the configured wall budget.
  (void)impl_->solve(a,a*p,5.);
}
Identifier::~Identifier()=default;
Fit Identifier::fit(const Eigen::MatrixXd & a,const Eigen::VectorXd & b,bool robust){
  auto start=Clock::now();
  if(a.cols()!=27 || a.rows()<30 || a.rows()%6 || b.size()!=a.rows() || !a.allFinite() || !b.allFinite())
    throw std::invalid_argument("Expected >=5 complete finite six-axis samples");
  auto remaining=[&](){double t=impl_->budget-elapsed(start);if(t<=0)throw std::runtime_error("Fit wall budget exceeded");return t;};
  Fit fit=impl_->solve(a,b,remaining()); double solver=fit.solver_seconds;
  if(robust){
    Eigen::VectorXd residual=a*fit.parameters-b,threshold(b.size());
    for(int axis=0;axis<6;++axis){
      std::vector<double> values;double magnitude=1;
      for(Eigen::Index i=axis;i<b.size();i+=6){values.push_back(residual[i]);magnitude=std::max(magnitude,std::abs(b[i]));}
      double center=median(values);
      for(auto & v:values)v=std::abs(v-center);
      double limit=1.5*std::max(median(values)/.6744897501960817,1e-10*magnitude);
      for(Eigen::Index i=axis;i<b.size();i+=6)threshold[i]=limit;
    }
    auto objective=[&](const Parameters & p){
      Eigen::VectorXd r=(a*p-b).cwiseAbs();double sum=0;
      for(Eigen::Index i=0;i<r.size();++i)sum+=r[i]<=threshold[i]?.5*r[i]*r[i]:threshold[i]*(r[i]-.5*threshold[i]);
      return sum;
    };
    double previous=objective(fit.parameters);bool converged=false;
    for(int iteration=1;iteration<=16;++iteration){
      remaining();residual=a*fit.parameters-b;
      Eigen::VectorXd weights=(threshold.array()/residual.cwiseAbs().array().max(std::numeric_limits<double>::min())).min(1.).sqrt();
      fit=impl_->solve(weights.asDiagonal()*a,weights.asDiagonal()*b,remaining());solver+=fit.solver_seconds;
      double current=objective(fit.parameters);
      double denominator=std::max(previous,std::numeric_limits<double>::min());
      fit.relative_change=std::abs(current-previous)/denominator;fit.iterations=iteration;
      remaining();
      if(iteration>=2 && fit.relative_change<1e-4 && (current-previous)/denominator<=1e-4){converged=true;break;}
      previous=current;
    }
    if(!converged)throw std::runtime_error("Huber IRLS did not converge");
  }
  Eigen::MatrixXd reduced=a*impl_->basis;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(reduced);
  double tolerance=std::numeric_limits<double>::epsilon()*std::max(reduced.rows(),reduced.cols())*svd.singularValues()[0];
  fit.rank=(svd.singularValues().array()>tolerance).count();
  fit.residual_rms=rms(a*fit.parameters-b); fit.solver_seconds=solver;
  remaining();fit.wall_seconds=elapsed(start);return fit;
}
}
