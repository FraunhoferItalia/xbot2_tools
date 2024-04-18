#ifndef TRAJECTORY_PLUGIN_H
#define TRAJECTORY_PLUGIN_H

#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>

namespace XBot {

class TrajectoryPlugin : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void starting() override;
    void run() override;
    void on_stop() override;

private:

    struct Chirp
    {
        double step(double dt);

        void reset();

        double warmup_time = 1.0;
        double freq_min_hz = 0.1;
        double freq_max_hz = 1.0;
        double period = 5.0;
        double amplitude = 0.0;
        double phase_offset = 0.0;
        double initial_value = 0.0;

    private:

        double _phase = 0;
        double _freq_hz = 0;
        double _time = 0;


    };

    std::map<Hal::JointBase*, Chirp> _trj_map;
};

}

#endif // TRAJECTORY_PLUGIN_H
