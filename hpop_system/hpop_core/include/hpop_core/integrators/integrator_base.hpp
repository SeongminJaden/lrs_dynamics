#pragma once

#include <array>
#include <functional>
#include <vector>
#include "hpop_core/state_vector.hpp"

namespace hpop_core
{

/// State type for 6-DOF integration [x, y, z, vx, vy, vz]
using State6 = std::array<double, 6>;

/// Derivative function type: f(t, state) -> derivative
using DerivativeFunc = std::function<State6(double, const State6&)>;

/// Integration result with error estimate
struct IntegrationResult
{
    State6 state;          // New state after integration
    double error{0.0};     // Error estimate (for adaptive integrators)
    bool success{true};    // Integration success flag
    int function_evals{0}; // Number of derivative evaluations
};

/// Integration step result for adaptive methods
struct AdaptiveStepResult
{
    State6 state;          // New state
    double dt_used;        // Actual step size used
    double dt_next;        // Recommended next step size
    double error;          // Error estimate
    int function_evals;    // Number of derivative evaluations
};

/// Base class for numerical integrators
class IntegratorBase
{
public:
    virtual ~IntegratorBase() = default;

    /// Perform single integration step
    virtual IntegrationResult step(const DerivativeFunc& f,
                                   double t,
                                   const State6& state,
                                   double dt) = 0;

    /// Get integrator name
    virtual const char* name() const = 0;

    /// Get order of integrator
    virtual int order() const = 0;

    /// Is this an adaptive integrator?
    virtual bool isAdaptive() const { return false; }

    /// Perform adaptive step (override for adaptive integrators)
    virtual AdaptiveStepResult adaptiveStep(const DerivativeFunc& f,
                                            double t,
                                            const State6& state,
                                            double dt,
                                            double tolerance)
    {
        auto result = step(f, t, state, dt);
        return {result.state, dt, dt, result.error, result.function_evals};
    }

protected:
    /// Helper: add two State6 arrays
    static State6 add(const State6& a, const State6& b)
    {
        State6 result;
        for (int i = 0; i < 6; ++i) result[i] = a[i] + b[i];
        return result;
    }

    /// Helper: multiply State6 by scalar
    static State6 scale(const State6& s, double factor)
    {
        State6 result;
        for (int i = 0; i < 6; ++i) result[i] = s[i] * factor;
        return result;
    }

    /// Helper: weighted sum of State6 arrays
    template<typename... Args>
    static State6 weightedSum(const std::vector<double>& weights,
                              const std::vector<State6>& states)
    {
        State6 result{0, 0, 0, 0, 0, 0};
        for (size_t i = 0; i < weights.size() && i < states.size(); ++i)
        {
            for (int j = 0; j < 6; ++j)
                result[j] += weights[i] * states[i][j];
        }
        return result;
    }

    /// Helper: compute error norm
    static double errorNorm(const State6& error, const State6& state)
    {
        double max_err = 0.0;
        for (int i = 0; i < 6; ++i)
        {
            double scale = std::abs(state[i]) + 1.0;  // Relative + absolute
            max_err = std::max(max_err, std::abs(error[i]) / scale);
        }
        return max_err;
    }
};

} // namespace hpop_core
