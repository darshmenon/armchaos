// armchaos_fast: relay /joint_states_raw -> /joint_states_chaos through the stream faults, in C++.
// Parameters:
//   source, out        topics (defaults /joint_states_raw, /joint_states_chaos)
//   faults             string array, one entry per fault: "at,for,name[,key=value...]", e.g. "2,6,freeze" or
//                      "1,5,spoof,joint=shoulder_pan_joint,offset=7"
//   lead               healthy seconds before scenario time 0 (default 0)
//   linger             seconds of clean relaying after the last fault ends before exiting (default 6)
//   max_duration       hard guard: exit after this many seconds in total (default 600)
// The `deactivate` fault (controller_manager) is not ported: use the Python armchaos for that.
#include <chrono>
#include <cstdio>
#include <sstream>

#include "armchaos_cpp/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using armchaos::Sample;

static std::vector<std::string> split(const std::string & s, char d) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string t;
  while (std::getline(ss, t, d)) out.push_back(t);
  return out;
}

class Relay : public rclcpp::Node {
 public:
  Relay() : Node("armchaos_fast") {
    auto source = declare_parameter<std::string>("source", "/joint_states_raw");
    auto out = declare_parameter<std::string>("out", "/joint_states_chaos");
    auto faults = declare_parameter<std::vector<std::string>>("faults", std::vector<std::string>{});
    lead_ = declare_parameter<double>("lead", 0.0);
    linger_ = declare_parameter<double>("linger", 6.0);
    max_duration_ = declare_parameter<double>("max_duration", 600.0);
    if (faults.empty()) throw std::invalid_argument("parameter 'faults' is empty, e.g. -p faults:=\"['2,6,freeze']\"");
    for (auto & f : faults) {
      auto tok = split(f, ',');
      if (tok.size() < 3) throw std::invalid_argument("fault '" + f + "' must be at,for,name[,key=value...]");
      armchaos::Params p;
      for (size_t i = 3; i < tok.size(); ++i) {
        auto e = tok[i].find('=');
        if (e == std::string::npos) throw std::invalid_argument("bad parameter '" + tok[i] + "' in '" + f + "'");
        p[tok[i].substr(0, e)] = tok[i].substr(e + 1);
      }
      if (tok[2] == "deactivate") throw std::invalid_argument("'deactivate' is not in the C++ relay; use the Python armchaos");
      engine_.add(std::stod(tok[0]), std::stod(tok[1]), armchaos::make_fault(tok[2], p));
      labels_.push_back(f);
    }
    t0_ = std::chrono::steady_clock::now();
    pub_ = create_publisher<sensor_msgs::msg::JointState>(out, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::JointState>(source, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          Sample s;
          s.names = m->name;
          s.pos = {m->position.begin(), m->position.end()};
          s.vel = {m->velocity.begin(), m->velocity.end()};
          s.eff = {m->effort.begin(), m->effort.end()};
          s.has_stamp = true; s.sec = m->header.stamp.sec; s.nsec = m->header.stamp.nanosec;
          try {
            emit(engine_.process(now_s(), s));
          } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "fault failed, sample dropped: %s", e.what());
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });
  }

 private:
  double elapsed() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count(); }
  double now_s() const { return elapsed() - lead_; }
  void emit(const std::vector<Sample> & v) {
    for (auto & s : v) {
      sensor_msgs::msg::JointState m;
      m.name = s.names;
      m.position = {s.pos.begin(), s.pos.end()};
      m.velocity = {s.vel.begin(), s.vel.end()};
      m.effort = {s.eff.begin(), s.eff.end()};
      if (s.has_stamp) { m.header.stamp.sec = s.sec; m.header.stamp.nanosec = s.nsec; }
      else m.header.stamp = now();
      pub_->publish(m);
    }
  }
  void tick() {
    const double t = now_s();
    if (elapsed() > max_duration_) { std::printf("stopping (max-duration)\n"); std::fflush(stdout); rclcpp::shutdown(); return; }
    auto [started, stopped] = engine_.update(t);
    for (auto & n : started) std::printf("t=%5.1fs START %s\n", t, n.c_str());
    for (auto & n : stopped) std::printf("t=%5.1fs STOP  %s\n", t, n.c_str());
    std::fflush(stdout);
    try { emit(engine_.tick(t)); } catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "%s", e.what()); }
    if (t > engine_.end() + 0.5 + linger_) rclcpp::shutdown();
  }
  armchaos::Engine engine_;
  std::vector<std::string> labels_;
  double lead_, linger_, max_duration_;
  std::chrono::steady_clock::time_point t0_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Relay>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "armchaos_fast: %s\n", e.what());
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
