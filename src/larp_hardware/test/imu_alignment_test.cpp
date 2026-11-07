#include <larp_hardware/imu_alignment.hpp>
#include <cassert>
using namespace larp_hardware;
int main(){
 ImuAlignment a; a.accel({1'000'000,{0,0,10}});a.gyro({3'000'000,{1,2,3}});
 assert(!a.pop());a.accel({5'000'000,{4,8,10}});auto m=a.pop();
 assert(m && m->ns==3'000'000 && m->acceleration[0]==2 && m->acceleration[1]==4 && m->gyro[2]==3);
 a.gyro({6'000'000,{1,2,3}});a.accel({50'000'000,{0,0,10}});assert(!a.pop());assert(a.dropped()==1);
 ImuAlignment b; b.accel({100,{1,2,3}});b.accel({100,{9,9,9}});b.gyro({100,{4,5,6}});
 assert(b.pop()->acceleration[0]==1);b.gyro({100,{7,8,9}});assert(!b.pop());
 // 250 Hz accelerometer and 200 Hz gyro preserve all bracketed gyro samples.
 ImuAlignment c;int count=0;
 for(int i=0;i<=100;++i){c.accel({1+i*4'000'000LL,{double(i*4),0,9.81}});if(i%5==0)for(int j=0;j<4;++j)c.gyro({1+(i/5*20+j*5)*1'000'000LL,{0,0,0}});while(auto x=c.pop()){assert(std::abs(x->acceleration[0]-(x->ns-1)/1e6)<1e-9);++count;}}
 assert(count==81);
}
