// example_controller.cpp -----------------------------------------------------
// A template low-level controller: a damper, tau = -kd·dq on every joint. docs/low-level-controllers.md
#include "low_level_controller/controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace example_controller
{
    using low_level_controller::MotorCommand;
    using low_level_controller::Parameters;
    using low_level_controller::State;

    class ExampleController : public low_level_controller::Controller
    {
    public:
        // controllers.example_controller.kd in the bridge config.
        bool configure(const Parameters &params, size_t, std::string &error) override
        {
            const auto kd = params.find("kd");
            if (kd == params.end() || kd->second.size() != 1 || !(kd->second[0] >= 0))
            {
                error = "needs controllers.example_controller.kd: one number >= 0";
                return false;
            }
            kd_ = static_cast<float>(kd->second[0]);
            return true;
        }

        // Each tick: no allocation, no blocking, no throw.
        void compute(const State &state, const std::vector<MotorCommand> &target,
                     std::vector<MotorCommand> &out) override
        {
            for (size_t i = 0; i < target.size(); ++i)
            {
                out[i] = target[i];   // q and dq unused at kp = kd = 0: the bridge sends the measured q
                out[i].kp = 0.0f;
                out[i].kd = 0.0f;
                out[i].tau = -kd_ * state.joints[i].dq;
            }
        }

    private:
        float kd_{0};
    };
} // namespace example_controller

PLUGINLIB_EXPORT_CLASS(example_controller::ExampleController, low_level_controller::Controller)
