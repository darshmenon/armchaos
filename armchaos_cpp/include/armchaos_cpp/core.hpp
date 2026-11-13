// ROS-free C++ port of the joint-state stream faults in armchaos/faults.py and the scheduling in armchaos/scenario.py.
// Same parameters, defaults and behaviour, so a C++ relay and the Python relay shape a stream the same way
// (armchaos/test/test_cpp_equivalence.py replays identical event sequences through both).
// Not ported on purpose: `deactivate` (controller_manager calls and the restore-on-exit safety stay in Python).
#pragma once
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace armchaos {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

struct Sample {
  std::vector<std::string> names;
  std::vector<double> pos, vel, eff;
  bool has_stamp = false;
  int32_t sec = 0;
  uint32_t nsec = 0;
};

using Params = std::map<std::string, std::string>;

class Fault {
 public:
  virtual ~Fault() = default;
  virtual const char * name() const = 0;
  virtual void reset() {}
  virtual std::vector<Sample> process(double, const Sample & s) { return {s}; }
  virtual std::vector<Sample> tick(double) { return {}; }

 protected:
  static double num(const Params & p, const char * k, double d) {
    auto it = p.find(k);
    return it == p.end() ? d : std::stod(it->second);
  }
  static std::string str(const Params & p, const char * k) {
    auto it = p.find(k);
    return it == p.end() ? "" : it->second;
  }
  static size_t idx(const std::string & joint, const Sample & s) {
    if (joint.empty()) return 0;
    for (size_t i = 0; i < s.names.size(); ++i)
      if (s.names[i] == joint) return i;
    throw std::runtime_error("joint '" + joint + "' not in the sample");
  }
};

struct Freeze : Fault {
  const char * name() const override { return "freeze"; }
  std::vector<Sample> process(double, const Sample &) override { return {}; }
};

struct Slow : Fault {
  double hz;
  std::optional<double> last;
  explicit Slow(const Params & p) : hz(num(p, "hz", 0.2)) {}
  const char * name() const override { return "slow"; }
  void reset() override { last.reset(); }
  std::vector<Sample> process(double now, const Sample & s) override {
    if (!last || now - *last >= 1.0 / hz) { last = now; return {s}; }
    return {};
  }
};

struct Stutter : Fault {
  double up, down;
  std::optional<double> t0;
  explicit Stutter(const Params & p) : up(num(p, "up", 1.0)), down(num(p, "down", 4.0)) {}
  const char * name() const override { return "stutter"; }
  void reset() override { t0.reset(); }
  std::vector<Sample> process(double now, const Sample & s) override {
    if (!t0) t0 = now;
    double period = up + down, m = std::fmod(now - *t0, period);
    if (m < 0) m += period;                                   // Python's % is non-negative
    return m < up ? std::vector<Sample>{s} : std::vector<Sample>{};
  }
};

struct Lag : Fault {
  double delay;
  std::deque<std::pair<double, Sample>> q;
  explicit Lag(const Params & p) : delay(num(p, "delay", 3.0)) {}
  const char * name() const override { return "lag"; }
  void reset() override { q.clear(); }
  std::vector<Sample> process(double now, const Sample & s) override { q.emplace_back(now + delay, s); return {}; }
  std::vector<Sample> tick(double now) override {
    std::vector<Sample> out;
    while (!q.empty() && q.front().first <= now) { out.push_back(std::move(q.front().second)); q.pop_front(); }
    return out;
  }
};

struct Noise : Fault {
  double sigma;
  std::string joint;
  uint64_t seed;
  std::mt19937_64 rng;
  std::normal_distribution<double> dist{0.0, 1.0};
  explicit Noise(const Params & p) : sigma(num(p, "sigma", 0.02)), joint(str(p, "joint")), seed((uint64_t)num(p, "seed", 0)) { reset(); }
  const char * name() const override { return "noise"; }
  void reset() override { rng.seed(seed); dist.reset(); }
  std::vector<Sample> process(double, const Sample & s) override {
    Sample o = s;
    for (size_t i = 0; i < o.pos.size(); ++i)
      if (joint.empty() || o.names[i] == joint) o.pos[i] += sigma * dist(rng);
    return {o};
  }
};

struct Nan : Fault {
  std::string joint;
  explicit Nan(const Params & p) : joint(str(p, "joint")) {}
  const char * name() const override { return "nan"; }
  std::vector<Sample> process(double, const Sample & s) override { Sample o = s; o.pos.at(idx(joint, s)) = NaN; return {o}; }
};

struct Spoof : Fault {
  std::string joint;
  double offset;
  explicit Spoof(const Params & p) : joint(str(p, "joint")), offset(num(p, "offset", 1.0)) {}
  const char * name() const override { return "spoof"; }
  std::vector<Sample> process(double, const Sample & s) override { Sample o = s; o.pos.at(idx(joint, s)) += offset; return {o}; }
};

struct Squeeze : Fault {
  std::string joint;
  double effort;
  explicit Squeeze(const Params & p) : joint(str(p, "joint")), effort(num(p, "effort", 100.0)) {}
  const char * name() const override { return "squeeze"; }
  std::vector<Sample> process(double, const Sample & s) override {
    Sample o = s;
    if (o.eff.empty()) o.eff.assign(o.pos.size(), 0.0);
    o.eff.at(idx(joint, s)) = effort;
    return {o};
  }
};

struct Spike : Fault {
  std::string joint;
  double velocity;
  explicit Spike(const Params & p) : joint(str(p, "joint")), velocity(num(p, "velocity", 10.0)) {}
  const char * name() const override { return "spike"; }
  std::vector<Sample> process(double, const Sample & s) override {
    Sample o = s;
    if (o.vel.empty()) o.vel.assign(o.pos.size(), 0.0);
    o.vel.at(idx(joint, s)) = velocity;
    return {o};
  }
};

inline const std::map<std::string, std::vector<std::string>> & fault_params() {
  static const std::map<std::string, std::vector<std::string>> m = {
      {"freeze", {}}, {"slow", {"hz"}}, {"stutter", {"up", "down"}}, {"lag", {"delay"}},
      {"noise", {"sigma", "joint", "seed"}}, {"nan", {"joint"}}, {"spoof", {"joint", "offset"}},
      {"squeeze", {"joint", "effort"}}, {"spike", {"joint", "velocity"}}};
  return m;
}

inline std::unique_ptr<Fault> make_fault(const std::string & name, const Params & p) {
  auto it = fault_params().find(name);
  if (it == fault_params().end()) throw std::invalid_argument("unknown fault '" + name + "'");
  for (auto & [k, v] : p) {
    bool ok = false;
    for (auto & a : it->second) ok |= (a == k);
    if (!ok) throw std::invalid_argument("fault '" + name + "' has no parameter '" + k + "'");
  }
  if (name == "freeze") return std::make_unique<Freeze>();
  if (name == "slow") return std::make_unique<Slow>(p);
  if (name == "stutter") return std::make_unique<Stutter>(p);
  if (name == "lag") return std::make_unique<Lag>(p);
  if (name == "noise") return std::make_unique<Noise>(p);
  if (name == "nan") return std::make_unique<Nan>(p);
  if (name == "spoof") return std::make_unique<Spoof>(p);
  if (name == "squeeze") return std::make_unique<Squeeze>(p);
  return std::make_unique<Spike>(p);
}

struct Step {
  double at, dur;
  std::unique_ptr<Fault> fault;
  bool active = false;
  double end() const { return at + dur; }
};

class Engine {
 public:
  void add(double at, double dur, std::unique_ptr<Fault> f) {
    if (at < 0 || dur <= 0) throw std::invalid_argument("'at' must be >= 0 and 'for' > 0");
    Step s{at, dur, std::move(f)};
    auto pos = steps_.begin();
    while (pos != steps_.end() && (*pos)->at <= at) ++pos;      // stable sort by `at`, like Python's sorted()
    steps_.insert(pos, std::make_unique<Step>(std::move(s)));
  }
  double end() const {
    double e = 0;
    for (auto & s : steps_) e = std::max(e, s->end());
    return e;
  }
  // Advance to scenario time t; returns the (started, stopped) faults' names.
  std::pair<std::vector<std::string>, std::vector<std::string>> update(double t) {
    std::pair<std::vector<std::string>, std::vector<std::string>> r;
    for (auto & s : steps_) {
      bool on = s->at <= t && t < s->end();
      if (on && !s->active) { s->fault->reset(); s->active = true; r.first.push_back(s->fault->name()); }
      else if (!on && s->active) { s->active = false; r.second.push_back(s->fault->name()); }
    }
    return r;
  }
  std::vector<Sample> process(double now, const Sample & s) { return pipe(now, {s}, 0); }
  std::vector<Sample> tick(double now) {
    std::vector<Sample> out;
    auto chain = active_chain();
    for (size_t i = 0; i < chain.size(); ++i) {
      auto part = pipe(now, chain[i]->tick(now), i + 1);
      out.insert(out.end(), part.begin(), part.end());
    }
    return out;
  }

 private:
  std::vector<Fault *> active_chain() {
    std::vector<Fault *> c;
    for (auto & s : steps_)
      if (s->active) c.push_back(s->fault.get());
    return c;
  }
  std::vector<Sample> pipe(double now, std::vector<Sample> samples, size_t start) {
    auto chain = active_chain();
    for (size_t i = start; i < chain.size(); ++i) {
      std::vector<Sample> nxt;
      for (auto & s : samples) {
        auto part = chain[i]->process(now, s);
        nxt.insert(nxt.end(), part.begin(), part.end());
      }
      samples = std::move(nxt);
    }
    return samples;
  }
  std::vector<std::unique_ptr<Step>> steps_;
};

}  // namespace armchaos
