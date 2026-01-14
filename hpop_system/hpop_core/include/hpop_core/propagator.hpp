#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <string>
#include <unordered_map>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "hpop_core/time_system.hpp"
#include "hpop_core/integrators/integrator_base.hpp"
#include "hpop_core/integrators/rk4.hpp"
#include "hpop_core/integrators/rkf78.hpp"

namespace hpop_core
{

/// Integrator type selection
enum class IntegratorType
{
    RK4,
    RKF78
};

/// Force model function: returns acceleration given position, velocity, and time
using ForceModelFunc = std::function<Vec3(const Vec3& pos, const Vec3& vel, double jd)>;

/// Satellite configuration
struct SatelliteConfig
{
    std::string id;               // Unique identifier
    std::string name;             // Human-readable name
    uint32_t norad_id{0};         // NORAD catalog number

    double mass{1000.0};          // [kg] Mass
    double area_drag{10.0};       // [m^2] Cross-section for drag
    double area_srp{10.0};        // [m^2] Cross-section for SRP
    double cd{2.2};               // [-] Drag coefficient
    double cr{1.5};               // [-] Reflectivity coefficient
};

/// Propagation result for a single step
struct PropagationStep
{
    StateVector state;
    Vec3 acceleration;
    double dt;
    double error;
};

/// Orbit propagator class
class OrbitPropagator
{
public:
    OrbitPropagator()
        : integrator_(std::make_unique<RKF78Integrator>())
    {
    }

    //=========================================================================
    // Configuration
    //=========================================================================

    /// Set integrator type
    void setIntegrator(IntegratorType type)
    {
        switch (type)
        {
            case IntegratorType::RK4:
                integrator_ = std::make_unique<RK4Integrator>();
                break;
            case IntegratorType::RKF78:
                integrator_ = std::make_unique<RKF78Integrator>();
                break;
        }
    }

    /// Set integration step size
    void setStepSize(double dt) { step_size_ = dt; }

    /// Set tolerance for adaptive integrators
    void setTolerance(double tol) { tolerance_ = tol; }

    /// Set central body gravitational parameter
    void setMu(double mu) { mu_ = mu; }

    /// Add force model
    void addForceModel(const std::string& name, ForceModelFunc model)
    {
        force_models_[name] = std::move(model);
    }

    /// Remove force model
    void removeForceModel(const std::string& name)
    {
        force_models_.erase(name);
    }

    /// Clear all force models except central gravity
    void clearForceModels()
    {
        force_models_.clear();
    }

    //=========================================================================
    // Propagation
    //=========================================================================

    /// Propagate state by duration (seconds)
    StateVector propagate(const StateVector& initial_state,
                         double duration,
                         std::vector<PropagationStep>* trajectory = nullptr)
    {
        StateVector state = initial_state;
        double t = 0.0;
        double dt = step_size_;

        while (t < duration)
        {
            if (t + dt > duration)
                dt = duration - t;

            // Create derivative function
            double jd = state.epoch + t / constants::SECONDS_PER_DAY;
            auto derivative = [this, jd](double, const State6& s) -> State6 {
                return computeDerivative(s, jd);
            };

            // Perform integration step
            State6 current_state = state.toArray();
            IntegrationResult result;

            if (integrator_->isAdaptive())
            {
                auto adaptive_result = integrator_->adaptiveStep(
                    derivative, t, current_state, dt, tolerance_);
                result.state = adaptive_result.state;
                result.error = adaptive_result.error;
                dt = adaptive_result.dt_next;
            }
            else
            {
                result = integrator_->step(derivative, t, current_state, dt);
            }

            // Update state
            state.fromArray(result.state);
            t += dt;

            // Record trajectory if requested
            if (trajectory)
            {
                PropagationStep step;
                step.state = state;
                step.state.epoch = initial_state.epoch + t / constants::SECONDS_PER_DAY;
                step.acceleration = computeTotalAcceleration(state.position, state.velocity, jd);
                step.dt = dt;
                step.error = result.error;
                trajectory->push_back(step);
            }
        }

        state.epoch = initial_state.epoch + duration / constants::SECONDS_PER_DAY;
        return state;
    }

    /// Propagate to specific Julian Date
    StateVector propagateToJD(const StateVector& initial_state, double target_jd)
    {
        double duration = (target_jd - initial_state.epoch) * constants::SECONDS_PER_DAY;
        return propagate(initial_state, duration);
    }

    /// Single propagation step
    PropagationStep singleStep(const StateVector& state, double dt)
    {
        double jd = state.epoch;
        auto derivative = [this, jd](double, const State6& s) -> State6 {
            return computeDerivative(s, jd);
        };

        State6 current_state = state.toArray();
        IntegrationResult result;

        if (integrator_->isAdaptive())
        {
            auto adaptive_result = integrator_->adaptiveStep(
                derivative, 0, current_state, dt, tolerance_);
            result.state = adaptive_result.state;
            result.error = adaptive_result.error;
        }
        else
        {
            result = integrator_->step(derivative, 0, current_state, dt);
        }

        PropagationStep step;
        step.state.fromArray(result.state);
        step.state.epoch = state.epoch + dt / constants::SECONDS_PER_DAY;
        step.acceleration = computeTotalAcceleration(step.state.position, step.state.velocity, jd);
        step.dt = dt;
        step.error = result.error;

        return step;
    }

    //=========================================================================
    // Utilities
    //=========================================================================

    /// Get current integrator name
    const char* integratorName() const { return integrator_->name(); }

    /// Compute total acceleration at given state
    Vec3 computeTotalAcceleration(const Vec3& pos, const Vec3& vel, double jd) const
    {
        // Central gravity (point mass)
        double r = pos.norm();
        Vec3 acc = pos * (-mu_ / (r * r * r));

        // Add perturbation force models
        for (const auto& [name, model] : force_models_)
        {
            acc += model(pos, vel, jd);
        }

        return acc;
    }

private:
    State6 computeDerivative(const State6& s, double jd) const
    {
        Vec3 pos{s[0], s[1], s[2]};
        Vec3 vel{s[3], s[4], s[5]};
        Vec3 acc = computeTotalAcceleration(pos, vel, jd);

        return {vel.x, vel.y, vel.z, acc.x, acc.y, acc.z};
    }

    std::unique_ptr<IntegratorBase> integrator_;
    std::unordered_map<std::string, ForceModelFunc> force_models_;

    double step_size_{60.0};      // [s] Default step size
    double tolerance_{1e-10};     // Tolerance for adaptive integrators
    double mu_{constants::EARTH_MU};  // Central body gravitational parameter
};

} // namespace hpop_core
