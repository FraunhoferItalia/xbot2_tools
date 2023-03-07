#ifndef XBOT2_TOOLS_MONITOR_H
#define XBOT2_TOOLS_MONITOR_H


#include <Eigen/Dense>
#include <memory>

namespace XBot {
namespace tools {

class Monitor
{

public:

    Monitor(std::string name);

    bool registerVariable(std::string name, double& var);

    bool registerVariable(std::string name, Eigen::VectorXd& var);

    bool publish();

    ~Monitor();

private:

    struct Impl;
    std::unique_ptr<Impl> i;


};

}
}

#endif // MONITOR_H
