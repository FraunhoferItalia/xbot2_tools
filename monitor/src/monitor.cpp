#include "monitor.h"

#include <pal_statistics_msgs/Statistics.h>
#include <pal_statistics_msgs/StatisticsNames.h>
#include <pal_statistics_msgs/StatisticsValues.h>
#include <xbot2/ros/ros_support.h>

using namespace XBot::tools;

class Monitor::Impl
{
public:
  Impl(std::string name);

  bool registerVariable(std::string name, double & var);

  bool registerVariable(std::string name, Eigen::VectorXd & var);

  bool publish();

private:
  PublisherPtr<pal_statistics_msgs::Statistics> _pub;
  PublisherPtr<pal_statistics_msgs::StatisticsNames> _pub_names;
  PublisherPtr<pal_statistics_msgs::StatisticsValues> _pub_values;

  std::vector<std::pair<std::string, double *>> _vars;
};

Monitor::Impl::Impl(std::string name)
{
  XBot::RosSupport ros;
  _pub = ros.advertise<pal_statistics_msgs::Statistics>(name + "/full", 10);
  _pub_names = ros.advertise<pal_statistics_msgs::StatisticsNames>(name + "/names", 10);
  _pub_values = ros.advertise<pal_statistics_msgs::StatisticsValues>(name + "/values", 10);

  if (!_pub) {
    throw std::runtime_error(fmt::format("[Monitor] could not advertise topic {}", name));
  }
}

bool Monitor::Impl::registerVariable(std::string name, double & var)
{
  _vars.emplace_back(name, &var);
  return true;
}

bool Monitor::Impl::registerVariable(std::string name, Eigen::VectorXd & var)
{
  bool success = true;

  for (int i = 0; i < var.size(); i++) {
    success = registerVariable(name + "_" + std::to_string(i), var[i]) && success;
  }

  return success;
}

bool Monitor::Impl::publish()
{
  auto msgptr = _pub->loanMessage();
  auto names_msgptr = _pub_names->loanMessage();
  auto values_msgptr = _pub_values->loanMessage();

  if (!msgptr || !names_msgptr || !values_msgptr) {
    std::cerr << "Unable to get loadned messages" << std::endl;
    return false;
  }

  auto & msg = **msgptr;
  auto & names_msg = **names_msgptr;
  auto & values_msg = **values_msgptr;

  auto now = XBot::chrono::system_clock::now();
  auto now_ts = XBot::chrono::duration_chrono_to_timespec(now.time_since_epoch());
  msg.header.stamp.sec = now_ts.tv_sec;
  msg.header.stamp.nsec = now_ts.tv_nsec;

  names_msg.header.stamp.sec = now_ts.tv_sec;
  names_msg.header.stamp.nsec = now_ts.tv_nsec;

  values_msg.header.stamp.sec = now_ts.tv_sec;
  values_msg.header.stamp.nsec = now_ts.tv_nsec;

  // std::cout << msg.statistics.size() <<std::endl;
  // std::cout << _vars.size() <<std::endl;
  if (msg.statistics.size() != _vars.size()) {
    // std::cout << "resizing" <<std::endl;
    msg.statistics.resize(_vars.size());
    names_msg.names.resize(_vars.size());
    values_msg.values.resize(_vars.size());

    for (int i = 0; i < _vars.size(); i++) {
      msg.statistics[i].name = _vars[i].first;
      names_msg.names[i] = _vars[i].first;
    }
  }

  for (int i = 0; i < _vars.size(); i++) {
    msg.statistics[i].value = *(_vars[i].second);
    values_msg.values[i] = *(_vars[i].second);
  }

  return _pub->publishLoaned(std::move(msgptr)) &&
         _pub_names->publishLoaned(std::move(names_msgptr)) &&
         _pub_values->publishLoaned(std::move(values_msgptr));
}

Monitor::Monitor(std::string name) { i = std::make_unique<Impl>(name); }

bool Monitor::registerVariable(std::string name, double & var)
{
  return i->registerVariable(name, var);
}

bool Monitor::registerVariable(std::string name, Eigen::VectorXd & var)
{
  return i->registerVariable(name, var);
}

bool Monitor::publish() { return i->publish(); }

Monitor::~Monitor() {}
