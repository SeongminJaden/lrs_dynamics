#pragma once

#include "hpop_core/integrators/integrator_base.hpp"
#include <cmath>
#include <algorithm>

namespace hpop_core
{

/// Runge-Kutta-Fehlberg 7(8) adaptive integrator
/// High precision integrator suitable for orbital mechanics
class RKF78Integrator : public IntegratorBase
{
public:
    const char* name() const override { return "RKF78"; }
    int order() const override { return 8; }
    bool isAdaptive() const override { return true; }

    IntegrationResult step(const DerivativeFunc& f,
                           double t,
                           const State6& state,
                           double dt) override
    {
        auto result = computeStep(f, t, state, dt);
        return {result.y8, result.error_norm, true, 13};
    }

    AdaptiveStepResult adaptiveStep(const DerivativeFunc& f,
                                    double t,
                                    const State6& state,
                                    double dt,
                                    double tolerance) override
    {
        const double safety = 0.9;
        const double min_scale = 0.1;
        const double max_scale = 5.0;

        AdaptiveStepResult result;
        result.function_evals = 0;

        double h = dt;
        int max_iterations = 20;

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            auto step_result = computeStep(f, t, state, h);
            result.function_evals += 13;

            double error = step_result.error_norm;

            if (error <= tolerance)
            {
                // Step accepted
                result.state = step_result.y8;
                result.dt_used = h;
                result.error = error;

                // Compute optimal next step size
                double scale = safety * std::pow(tolerance / std::max(error, 1e-15), 1.0 / 8.0);
                scale = std::clamp(scale, min_scale, max_scale);
                result.dt_next = h * scale;

                return result;
            }

            // Step rejected, reduce step size
            double scale = safety * std::pow(tolerance / error, 1.0 / 8.0);
            scale = std::max(scale, min_scale);
            h *= scale;
        }

        // Failed to converge
        result.state = state;
        result.dt_used = 0;
        result.dt_next = dt * 0.1;
        result.error = -1;  // Indicate failure
        return result;
    }

private:
    struct StepResult
    {
        State6 y7;          // 7th order solution
        State6 y8;          // 8th order solution
        double error_norm;  // Error estimate
    };

    StepResult computeStep(const DerivativeFunc& f,
                           double t,
                           const State6& state,
                           double h)
    {
        // RKF78 coefficients (Fehlberg)
        // Node points (c_i)
        constexpr double c2 = 2.0 / 27.0;
        constexpr double c3 = 1.0 / 9.0;
        constexpr double c4 = 1.0 / 6.0;
        constexpr double c5 = 5.0 / 12.0;
        constexpr double c6 = 1.0 / 2.0;
        constexpr double c7 = 5.0 / 6.0;
        constexpr double c8 = 1.0 / 6.0;
        constexpr double c9 = 2.0 / 3.0;
        constexpr double c10 = 1.0 / 3.0;
        constexpr double c11 = 1.0;
        constexpr double c12 = 0.0;
        constexpr double c13 = 1.0;

        // Stage evaluations
        State6 k1 = f(t, state);

        State6 y2;
        for (int i = 0; i < 6; ++i)
            y2[i] = state[i] + h * c2 * k1[i];
        State6 k2 = f(t + c2 * h, y2);

        State6 y3;
        for (int i = 0; i < 6; ++i)
            y3[i] = state[i] + h * (k1[i] / 36.0 + k2[i] / 12.0);
        State6 k3 = f(t + c3 * h, y3);

        State6 y4;
        for (int i = 0; i < 6; ++i)
            y4[i] = state[i] + h * (k1[i] / 24.0 + k3[i] / 8.0);
        State6 k4 = f(t + c4 * h, y4);

        State6 y5;
        for (int i = 0; i < 6; ++i)
            y5[i] = state[i] + h * (5.0 / 12.0 * k1[i] - 25.0 / 16.0 * k3[i] + 25.0 / 16.0 * k4[i]);
        State6 k5 = f(t + c5 * h, y5);

        State6 y6;
        for (int i = 0; i < 6; ++i)
            y6[i] = state[i] + h * (1.0 / 20.0 * k1[i] + 1.0 / 4.0 * k4[i] + 1.0 / 5.0 * k5[i]);
        State6 k6 = f(t + c6 * h, y6);

        State6 y7;
        for (int i = 0; i < 6; ++i)
            y7[i] = state[i] + h * (-25.0 / 108.0 * k1[i] + 125.0 / 108.0 * k4[i]
                                   - 65.0 / 27.0 * k5[i] + 125.0 / 54.0 * k6[i]);
        State6 k7 = f(t + c7 * h, y7);

        State6 y8;
        for (int i = 0; i < 6; ++i)
            y8[i] = state[i] + h * (31.0 / 300.0 * k1[i] + 61.0 / 225.0 * k5[i]
                                   - 2.0 / 9.0 * k6[i] + 13.0 / 900.0 * k7[i]);
        State6 k8 = f(t + c8 * h, y8);

        State6 y9;
        for (int i = 0; i < 6; ++i)
            y9[i] = state[i] + h * (2.0 * k1[i] - 53.0 / 6.0 * k4[i] + 704.0 / 45.0 * k5[i]
                                   - 107.0 / 9.0 * k6[i] + 67.0 / 90.0 * k7[i] + 3.0 * k8[i]);
        State6 k9 = f(t + c9 * h, y9);

        State6 y10;
        for (int i = 0; i < 6; ++i)
            y10[i] = state[i] + h * (-91.0 / 108.0 * k1[i] + 23.0 / 108.0 * k4[i]
                                    - 976.0 / 135.0 * k5[i] + 311.0 / 54.0 * k6[i]
                                    - 19.0 / 60.0 * k7[i] + 17.0 / 6.0 * k8[i] - 1.0 / 12.0 * k9[i]);
        State6 k10 = f(t + c10 * h, y10);

        State6 y11;
        for (int i = 0; i < 6; ++i)
            y11[i] = state[i] + h * (2383.0 / 4100.0 * k1[i] - 341.0 / 164.0 * k4[i]
                                    + 4496.0 / 1025.0 * k5[i] - 301.0 / 82.0 * k6[i]
                                    + 2133.0 / 4100.0 * k7[i] + 45.0 / 82.0 * k8[i]
                                    + 45.0 / 164.0 * k9[i] + 18.0 / 41.0 * k10[i]);
        State6 k11 = f(t + c11 * h, y11);

        State6 y12;
        for (int i = 0; i < 6; ++i)
            y12[i] = state[i] + h * (3.0 / 205.0 * k1[i] - 6.0 / 41.0 * k6[i]
                                    - 3.0 / 205.0 * k7[i] - 3.0 / 41.0 * k8[i]
                                    + 3.0 / 41.0 * k9[i] + 6.0 / 41.0 * k10[i]);
        State6 k12 = f(t + c12 * h, y12);

        State6 y13;
        for (int i = 0; i < 6; ++i)
            y13[i] = state[i] + h * (-1777.0 / 4100.0 * k1[i] - 341.0 / 164.0 * k4[i]
                                    + 4496.0 / 1025.0 * k5[i] - 289.0 / 82.0 * k6[i]
                                    + 2193.0 / 4100.0 * k7[i] + 51.0 / 82.0 * k8[i]
                                    + 33.0 / 164.0 * k9[i] + 12.0 / 41.0 * k10[i] + k12[i]);
        State6 k13 = f(t + c13 * h, y13);

        // 7th order solution (y7)
        StepResult result;
        for (int i = 0; i < 6; ++i)
        {
            result.y7[i] = state[i] + h * (41.0 / 840.0 * k1[i] + 34.0 / 105.0 * k6[i]
                                          + 9.0 / 35.0 * k7[i] + 9.0 / 35.0 * k8[i]
                                          + 9.0 / 280.0 * k9[i] + 9.0 / 280.0 * k10[i]
                                          + 41.0 / 840.0 * k11[i]);
        }

        // 8th order solution (y8)
        for (int i = 0; i < 6; ++i)
        {
            result.y8[i] = state[i] + h * (34.0 / 105.0 * k6[i] + 9.0 / 35.0 * k7[i]
                                          + 9.0 / 35.0 * k8[i] + 9.0 / 280.0 * k9[i]
                                          + 9.0 / 280.0 * k10[i] + 41.0 / 840.0 * k12[i]
                                          + 41.0 / 840.0 * k13[i]);
        }

        // Error estimate
        State6 error;
        for (int i = 0; i < 6; ++i)
            error[i] = result.y8[i] - result.y7[i];

        result.error_norm = errorNorm(error, result.y8);

        return result;
    }
};

} // namespace hpop_core
