#include <larp_hardware/command_gate.hpp>
#include <cassert>
using namespace larp_hardware;
int main(){CommandGate g;CommandSample s{100,200,{1,2,3},{0,0,0},{0,0,0},0,0};
 assert(g.accept(s,100));assert(g.get(199,true));assert(!g.get(200,true));assert(!g.get(150,false));assert(!g.get(99,true));
 s.deadline_steady_ns=400'000'000;assert(!g.accept(s,100));assert(!g.get(101,true));
 s.deadline_steady_ns=200;s.velocity[0]=NAN;assert(!g.accept(s,100));g.clear();assert(!g.get(101,true));}
