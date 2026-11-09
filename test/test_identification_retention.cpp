#include "ros2_control_blue_reach_5/vehicle_identifier.hpp"
#include "ros2_control_blue_reach_5/identification_sample.hpp"
#include <gtest/gtest.h>
#include <random>

namespace vi = uvms::identification;
TEST(IdentificationWindow, GeneratedRestoringIsSubmergedAtEveryDepth) {
  Eigen::MatrixXd reference;
  for (double depth : {-1., -.001, 0., .001, 1., 3., 10.}) {
    vi::LiveWindow window(5, .05);
    for (int step = 0; step <= 6; ++step) {
      vi::Measurement m{0., double(step), step*.01, .01, depth,
                        Eigen::Vector3d::Zero(), vi::Vector6::Zero(), vi::Vector6::Zero()};
      window.push(m);
    }
    const auto result = window.snapshot();
    ASSERT_TRUE(result);
    // FRD restoring force is B-W in heave. Coordinate zero is not a surface switch.
    EXPECT_NEAR(result->design(2, 10), -1., 1e-14);
    EXPECT_NEAR(result->design(2, 11), 1., 1e-14);
    EXPECT_LT((result->design.col(10)+result->design.col(11)).norm(), 1e-14);
    if (reference.size()==0) reference=result->design;
    EXPECT_LT((result->design-reference).norm(), 1e-14);
  }
}

TEST(HardwareIdentification, IntegratesAppliedWrenchBetweenFreshMeasurements) {
  vi::HardwareSample s;
  const std::array<double,6> a{1,2,3,4,5,6}, b{3,6,9,12,15,18};
  ASSERT_TRUE(s.advance(.01,a));ASSERT_TRUE(s.observe(2,{.1,.2,.3},a));
  const auto first_time=s.time;
  ASSERT_TRUE(s.advance(.1,a));
  EXPECT_EQ(s.step,1);EXPECT_EQ(s.time,first_time); // cached packet, not a new observation
  ASSERT_TRUE(s.advance(.2,b));ASSERT_TRUE(s.observe(3,{.4,.5,.6},b));
  EXPECT_EQ(s.step,2);EXPECT_NEAR(s.period,.3,1e-12);
  EXPECT_NEAR(s.time-first_time,s.period,1e-12);
  for(int i=0;i<6;++i)EXPECT_NEAR(s.wrench[i],(a[i]+2*b[i])/3,1e-12);
  EXPECT_EQ(s.velocity,b);EXPECT_DOUBLE_EQ(s.depth,3);
}
TEST(HardwareIdentification, GapsAndInvalidSensorsInvalidateAndReanchor) {
  vi::HardwareSample s;std::array<double,6> v{};
  s.advance(.1,v);s.observe(2,{0,0,0},v);
  const auto epoch=s.generation;
  EXPECT_FALSE(s.advance(.6,v));EXPECT_EQ(s.valid,0);EXPECT_GT(s.generation,epoch);
  ASSERT_TRUE(s.advance(.1,v));ASSERT_TRUE(s.observe(2,{0,0,0},v));
  EXPECT_EQ(s.step,1); // next receiver packet resets the old interval
  v[2]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(s.observe(2,{0,0,0},v));EXPECT_EQ(s.valid,0);
}
TEST(HardwareIdentification, MissingFreshSensorsCannotAccumulateAnUnboundedInterval) {
  vi::HardwareSample s;std::array<double,6> v{};
  s.advance(.1,v);s.observe(2,{0,0,0},v);
  for(int i=0;i<5;++i)ASSERT_TRUE(s.advance(.1,v));
  EXPECT_FALSE(s.advance(.1,v));EXPECT_EQ(s.valid,0);
}
namespace {
vi::Window window(double start, const vi::Parameters & parameters, double strength=1.) {
  vi::Window w{0,start,start+10,Eigen::MatrixXd(606,27),{}, {}};
  std::mt19937 generator(42);
  std::normal_distribution<double> normal;
  for(int r=0;r<606;++r)for(int c=0;c<27;++c)
    w.design(r,c)=strength*normal(generator);
  w.design.col(11)=-w.design.col(10); // Submerged W-B, not individually W and B.
  w.wrench=w.design*parameters;
  for(int i=0;i<101;++i)w.times.push_back(start+i*.1);
  return w;
}
vi::Fit fit(const vi::Parameters & p) {
  vi::Fit f;f.parameters=p;f.residual_rms.setZero();return f;
}
}

TEST(IdentificationRetention, FixedScalingDoesNotManufactureExcitation) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();
  auto info=retention.information(window(0,p).design);
  EXPECT_TRUE(info.sufficient);EXPECT_EQ(info.dimensions,24);EXPECT_TRUE(info.net_weight_only);
  EXPECT_FALSE(retention.information(window(0,p,1e-10).design).sufficient);
  EXPECT_FALSE(retention.information(window(0,p,0).design).sufficient);
}

TEST(IdentificationRetention, StrongColumnDoesNotInvalidateWeakerObservableDirections) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();
  auto w=window(0,p,.001);
  ASSERT_TRUE(retention.information(w.design).sufficient);
  // Increase only the already strong W-B direction, not the weak directions.
  w.design.col(10)*=1e6;w.design.col(11)*=1e6;
  auto information=retention.information(w.design);
  EXPECT_GT(information.condition,1e8);
  EXPECT_TRUE(information.sufficient);EXPECT_EQ(information.rank,24);
}

TEST(IdentificationRetention, QuietWindowsDoNotInvokeSolver) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();int calls=0;
  auto solve=[&](const auto &,const auto &){++calls;return fit(p);};
  EXPECT_FALSE(retention.update(window(0,p,0),solve).candidate);
  EXPECT_EQ(calls,0);
  retention.update(window(11,p),solve);
  ASSERT_TRUE(retention.update(window(22,p),solve).valid);
  int before=calls;
  auto result=retention.update(window(33,p,0),solve);
  EXPECT_EQ(calls,before);EXPECT_FALSE(result.candidate);EXPECT_TRUE(result.valid);
}

TEST(IdentificationRetention, StopHoldsAndIndependentMismatchInvalidates) {
  vi::ModelRetention retention({},false);
  vi::Parameters p=vi::Parameters::Ones();
  auto solve=[&](const auto &,const auto &){return fit(p);};
  auto w=window(0,p);
  auto result=retention.update(w,solve);
  EXPECT_TRUE(result.candidate);EXPECT_FALSE(result.available);
  for(int i=0;i<5;++i)EXPECT_FALSE(retention.update(w,solve).available);
  result=retention.update(window(11,p),solve);
  ASSERT_TRUE(result.accepted);ASSERT_TRUE(result.valid);
  for(int i=0;i<20;++i) {
    result=retention.update(window(22+11*i,p,0),solve);
    EXPECT_TRUE(result.valid);EXPECT_EQ(result.parameters,p);
  }
  for(int i=0;i<3;++i) {
    auto bad=window(250+11*i,p,0);bad.wrench.setOnes();
    result=retention.update(bad,solve);EXPECT_EQ(result.contradictions,i+1);
    result=retention.update(bad,solve);EXPECT_EQ(result.contradictions,i+1);
  }
  EXPECT_TRUE(result.available);EXPECT_FALSE(result.valid);
  p*=2;
  EXPECT_FALSE(retention.update(window(283,p),solve).valid);
  result=retention.update(window(294,p),solve);
  EXPECT_TRUE(result.accepted);EXPECT_TRUE(result.valid);EXPECT_EQ(result.parameters,p);
  auto reset=window(305,p,0);reset.epoch=1;
  EXPECT_FALSE(retention.update(reset,solve).available);
}

TEST(IdentificationRetention, BadLaterPredictionAndQuietValidationCannotApprove) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();
  auto wrong=[&](const auto &,const auto &){return fit(2*p);};
  retention.update(window(0,p),wrong);
  EXPECT_FALSE(retention.update(window(11,p,0),wrong).available);
  EXPECT_FALSE(retention.update(window(22,p),wrong).available);
}

TEST(IdentificationRetention, SolverFailurePreservesRetainedModel) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();
  auto solve=[&](const auto &,const auto &){return fit(p);};
  retention.update(window(0,p),solve);
  ASSERT_TRUE(retention.update(window(11,p),solve).valid);
  auto fail=[](const auto &,const auto &)->vi::Fit {throw std::runtime_error("solver unavailable");};
  auto result=retention.update(window(22,p),fail);
  EXPECT_TRUE(result.valid);EXPECT_EQ(result.parameters,p);
  EXPECT_EQ(result.detail,"solver unavailable");
}

TEST(IdentificationRetention, SubmergedFitIsNotApprovedForDifferentBuoyancyRegime) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();
  auto solve=[&](const auto &,const auto &){return fit(p);};
  retention.update(window(0,p),solve);
  ASSERT_TRUE(retention.update(window(11,p),solve).valid);
  auto surface=window(22,p);
  surface.design.col(11)*=.5;
  surface.wrench=surface.design*p;
  auto result=retention.update(surface,solve);
  EXPECT_TRUE(result.available);EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.retained_net_weight_only);
  EXPECT_EQ(result.status,"retained_outside_identified_regime");
}

TEST(IdentificationRetention, RetainedRmsUsesRecentPostAcceptanceMeasurementsAtRest) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();
  auto solve=[&](const auto &,const auto &){return fit(p);};
  EXPECT_FALSE(retention.update(window(0,p),solve).retained_rms_available);
  auto accepted=retention.update(window(11,p),solve);
  ASSERT_TRUE(accepted.accepted);
  EXPECT_FALSE(accepted.retained_rms_available);
  auto fresh=window(15,p,0); // Acceptance time 21 is inside this window.
  for(std::size_t i=0;i<fresh.times.size();++i)
    for(int axis=0;axis<6;++axis)
      fresh.wrench[6*i+axis]=fresh.times[i]<=21 ? 1000. : axis+1.;
  auto result=retention.update(fresh,solve);
  ASSERT_TRUE(result.retained_rms_available);
  EXPECT_EQ(result.retained_rms_samples,40u);
  EXPECT_GT(result.retained_rms_start,21);
  EXPECT_DOUBLE_EQ(result.retained_rms_end,25);
  for(int axis=0;axis<6;++axis)EXPECT_DOUBLE_EQ(result.retained_rms[axis],axis+1.);
  // The rolling monitor updates even without a new validation vote.
  fresh=window(16,p,0);fresh.wrench.setConstant(2.);
  result=retention.update(fresh,solve);
  EXPECT_TRUE(result.retained_rms_available);
  EXPECT_EQ(result.retained_rms_samples,50u);
  EXPECT_DOUBLE_EQ(result.retained_rms[0],2.);
  fresh.epoch=1;
  EXPECT_FALSE(retention.update(fresh,solve).retained_rms_available);
}

TEST(IdentificationWindow, QuadraticDragIntegralAndInputSwitchFallback) {
  for(bool switched : {false,true}) {
    vi::LiveWindow live(5,.05);
    const std::array<double,7> times{0.,.01,.025,.035,.05,.06,.075};
    for(int i=0;i<7;++i) {
      double h=i ? times[i]-times[i-1] : .01;
      vi::Measurement m{0.,double(i),times[i],h,2.,Eigen::Vector3d::Zero(),
                         vi::Vector6::Zero(),vi::Vector6::Constant(switched ? i : 2.)};
      m.velocity[0]=1.+times[i];live.push(m);
    }
    auto w=live.snapshot();ASSERT_TRUE(w);
    ASSERT_EQ(w->support_starts.size(),5u);
    for(int row=0;row<5;++row) {
      double a=times[row], b=times[row+1], c=times[row+2];
      EXPECT_DOUBLE_EQ(w->support_starts[row],a);
      double exact=1.+a+c+(a*a+a*c+c*c)/3.;
      double trap=((b-a)*((1+a)*(1+a)+(1+b)*(1+b))+
                   (c-b)*((1+b)*(1+b)+(1+c)*(1+c)))/(2*(c-a));
      EXPECT_NEAR(w->design(row*6,21),-(switched ? trap : exact),1e-13);
      EXPECT_NEAR(w->design(row*6,0),1.,1e-13);
      double input=switched ? ((b-a)*(row+1)+(c-b)*(row+2))/(c-a) : 2.;
      EXPECT_NEAR(w->wrench[row*6],input,1e-13);
    }
    live.reset();EXPECT_EQ(live.size(),0u);EXPECT_FALSE(live.snapshot());
  }
}

TEST(IdentificationRetention, ValidationExcludesRowsWhoseSupportOverlapsTraining) {
  vi::ModelRetention retention({},false);
  auto p=vi::Parameters::Ones().eval();
  auto solve=[&](const auto &,const auto &){return fit(p);};
  retention.update(window(0,p),solve); // training ends at 10
  auto overlap=window(11,p);
  // Row timestamps look later, but every row still depends on training data.
  for(double t:overlap.times)overlap.support_starts.push_back(t-12.);
  EXPECT_FALSE(retention.update(overlap,solve).accepted);
  EXPECT_TRUE(retention.update(window(30,p),solve).accepted);
}
