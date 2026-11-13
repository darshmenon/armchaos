// Replay an event sequence through the C++ engine and print every emitted sample (for the equivalence test).
//   F at for name k=v ...   add a fault           U t   update the schedule
//   S t n name pos vel eff ... (counts: n joints, vel/eff given as "-" for none)   process a sample
//   T t   tick (delayed output)
// Output: "O n name pos vel eff ..." per emitted sample (vel/eff empty => "-"), "ERR msg" if a fault throws, "--" after each event.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include "armchaos_cpp/core.hpp"
using namespace armchaos;

static void emit(const std::vector<Sample> & v) {
  for (auto & s : v) {
    std::printf("O %zu", s.names.size());
    for (size_t i = 0; i < s.names.size(); ++i) {
      std::printf(" %s %.17g ", s.names[i].c_str(), s.pos[i]);
      s.vel.empty() ? std::printf("-") : std::printf("%.17g", s.vel[i]);
      std::printf(" ");
      s.eff.empty() ? std::printf("-") : std::printf("%.17g", s.eff[i]);
    }
    std::printf("\n");
  }
}

int main(int argc, char ** argv) {
  if (argc != 2) return 2;
  std::ifstream in(argv[1]);
  Engine eng;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream is(line);
    std::string tag;
    is >> tag;
    try {
      if (tag == "F") {
        double at, dur;
        std::string name, kv;
        is >> at >> dur >> name;
        Params p;
        while (is >> kv) { auto e = kv.find('='); p[kv.substr(0, e)] = kv.substr(e + 1); }
        eng.add(at, dur, make_fault(name, p));
        continue;
      }
      if (tag == "U") { double t; is >> t; eng.update(t); }
      else if (tag == "S") {
        double t;
        int n;
        is >> t >> n;
        Sample s;
        for (int i = 0; i < n; ++i) {
          std::string nm, p, v, e;
          is >> nm >> p >> v >> e;
          s.names.push_back(nm);
          s.pos.push_back(std::stod(p));
          if (v != "-") s.vel.push_back(std::stod(v));
          if (e != "-") s.eff.push_back(std::stod(e));
        }
        emit(eng.process(t, s));
      } else if (tag == "T") { double t; is >> t; emit(eng.tick(t)); }
    } catch (const std::exception & ex) {
      std::printf("ERR %s\n", ex.what());
    }
    std::printf("--\n");
  }
  return 0;
}
