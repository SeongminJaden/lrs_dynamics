#pragma once

#include "hpop_core/integrators/integrator_base.hpp"

namespace hpop_core
{

/// Classic 4th order Runge-Kutta integrator
class RK4Integrator : public IntegratorBase
{
public:
    const char* name() const override { return "RK4"; }
    int order() const override { return 4; }

    IntegrationResult step(const DerivativeFunc& f,
                           double t,
                           const State6& state,
                           double dt) override
    {
        IntegrationResult result;
        result.function_evals = 4;

        // k1 = f(t, y)
        State6 k1 = f(t, state);

        // k2 = f(t + dt/2, y + dt/2 * k1)
        State6 y2 = add(state, scale(k1, dt / 2.0));
        State6 k2 = f(t + dt / 2.0, y2);

        // k3 = f(t + dt/2, y + dt/2 * k2)
        State6 y3 = add(state, scale(k2, dt / 2.0));
        State6 k3 = f(t + dt / 2.0, y3);

        // k4 = f(t + dt, y + dt * k3)
        State6 y4 = add(state, scale(k3, dt));
        State6 k4 = f(t + dt, y4);

        // y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        for (int i = 0; i < 6; ++i)
        {
            result.state[i] = state[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }

        result.success = true;
        return result;
    }
};

} // namespace hpop_core
