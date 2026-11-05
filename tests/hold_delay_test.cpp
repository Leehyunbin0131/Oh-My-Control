// Actual control core against a delayed, unmeasured plant: regression only.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include "../firmware/mega_motor_test/velocity_core.h"

int main(int argc,char** argv) {
  assert(argc==7);
  const int kp=std::atoi(argv[1]),ki=std::atoi(argv[2]),position=std::atoi(argv[3]);
  const int outer=std::atoi(argv[4]),limit=std::atoi(argv[5]),band=std::atoi(argv[6]);
  unsigned cases=0; int worstLoaded=0,worstFinal=0;
  for(int delay:{1,2,3,5,7,10}) for(double friction:{20.,25.,35.,45.,55.})
    for(double tau:{.04,.06,.12,.18,.3,.4}) for(double slope:{5.5,6.3,7.5})
      for(int scenario:{2,20,200}) for(int direction:{-1,1}) {
        VelocityControl v; uint32_t count[2]={0,0},invalid[2]={0,0},now=0;
        double rpm=0,x=0,load=0; std::vector<int> history{0};
        for(int i=0;i<2;++i) {
          assert(v.configure(i,1,1.5,8,0,6.3,35));
          assert(v.configure(i,-1,1.5,8,0,6.3,35));
          assert(v.configureZero(i,2000,0,150));
          assert(v.configureHoldPI(i,kp,ki));
          assert(v.configureHoldPosition(i,position));
          assert(v.configureHold(i,true,outer,500,limit,band,100,0,1000));
        }
        auto step=[&]() {
          ++now;
          const double effort=(now>=static_cast<unsigned>(delay)?history[now-delay]:0)+load;
          const double driven=std::copysign(std::max(0.,std::abs(effort)-friction)/slope,effort);
          rpm+=.001/tau*(driven-rpm); x+=rpm*2652/60000;
          count[0]=static_cast<uint32_t>(static_cast<int32_t>(std::round(x)));
          const int fault=v.update(count,invalid,now);
          if(fault) std::cerr<<"fault "<<fault<<" delay "<<delay<<" scenario "<<scenario<<'\n';
          assert(!fault&&std::abs(v.output[0])<=limit);
          history.push_back(v.output[0]);
        };
        v.start(0,0,count,invalid,0);
        for(int i=0;i<200;++i) step();
        if(scenario==200) load=direction*200;
        else for(int i=0;i<scenario;++i) { x+=direction; step(); }
        for(int i=0;i<2500;++i) {
          step();
          if(scenario==200) worstLoaded=std::max(worstLoaded,std::abs(VelocityControl::delta(count[0],0)));
        }
        load=0;
        for(int i=0;i<12000;++i) step();
        int lastSign=0,flips=0,lo=10000,hi=-10000;
        for(int i=0;i<3000;++i) {
          step(); const int e=VelocityControl::delta(count[0],0);
          lo=std::min(lo,e);hi=std::max(hi,e);
          const int s=VelocityControl::sign(v.output[0]);
          if(s&&lastSign&&s!=lastSign) ++flips;
          if(s) lastSign=s;
        }
        if(flips||lo!=hi||std::abs(lo)>3)
          std::cerr<<"unsettled delay="<<delay<<" f="<<friction<<" tau="<<tau<<" slope="<<slope
                   <<" scenario="<<scenario<<" range="<<lo<<','<<hi<<" flips="<<flips<<'\n';
        assert(!flips&&lo==hi&&std::abs(lo)<=3);
        assert(v.holdReference[0]==0&&v.state[0]==VelocityControl::HOLD);
        worstFinal=std::max(worstFinal,std::abs(lo)); ++cases;
      }
  assert(worstLoaded<=25);
  std::cout<<cases<<" delayed HOLD model cases passed; peak external-load error "<<worstLoaded
           <<" counts, final fixed error "<<worstFinal<<" counts; unmeasured plant\n";
}
