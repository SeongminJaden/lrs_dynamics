/**
 * @file lambert_solver.hpp
 * @brief Lambert problem solver for orbit determination and rendezvous
 *
 * Solves the boundary value problem: given two position vectors and
 * time of flight, find the orbit that connects them.
 *
 * Uses Izzo's algorithm (fast and robust).
 */

#ifndef HPOP_MANEUVER_LAMBERT_SOLVER_HPP
#define HPOP_MANEUVER_LAMBERT_SOLVER_HPP

#include <cmath>
#include <vector>
#include <stdexcept>
#include <algorithm>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"

namespace hpop_maneuver
{

/**
 * @brief Lambert problem solution
 */
struct LambertSolution
{
    hpop_core::Vec3 v1;        // Velocity at r1 [m/s]
    hpop_core::Vec3 v2;        // Velocity at r2 [m/s]
    double dv1;                 // Delta-v at departure [m/s]
    double dv2;                 // Delta-v at arrival [m/s]
    double total_dv;            // Total delta-v [m/s]
    int revolutions;            // Number of complete revolutions
    bool is_prograde;           // Direction of transfer
    bool converged;             // Solution found

    LambertSolution()
        : dv1(0), dv2(0), total_dv(0), revolutions(0)
        , is_prograde(true), converged(false) {}
};

/**
 * @brief Lambert problem solver (Izzo's algorithm)
 */
class LambertSolver
{
public:
    /**
     * @brief Solve Lambert's problem
     *
     * @param r1 Initial position [m]
     * @param r2 Final position [m]
     * @param tof Time of flight [s]
     * @param mu Gravitational parameter [m^3/s^2]
     * @param prograde Prograde (true) or retrograde (false) transfer
     * @param multi_rev Number of complete revolutions (0 for short way)
     * @return LambertSolution
     */
    static LambertSolution solve(const hpop_core::Vec3& r1,
                                 const hpop_core::Vec3& r2,
                                 double tof,
                                 double mu = hpop_core::constants::EARTH_MU,
                                 bool prograde = true,
                                 int multi_rev = 0)
    {
        LambertSolution sol;
        sol.is_prograde = prograde;
        sol.revolutions = multi_rev;

        // Magnitudes
        double r1_mag = r1.norm();
        double r2_mag = r2.norm();

        // Chord and semi-perimeter
        hpop_core::Vec3 c_vec = r2 - r1;
        double c = c_vec.norm();
        double s = (r1_mag + r2_mag + c) / 2.0;

        // Direction of motion
        hpop_core::Vec3 h = r1.cross(r2);
        double lambda_sq = 1.0 - c / s;
        double lambda = std::sqrt(lambda_sq);

        // Check transfer angle
        if ((prograde && h.z < 0) || (!prograde && h.z > 0))
        {
            lambda = -lambda;
        }

        // Non-dimensional time of flight
        double T = std::sqrt(2.0 * mu / (s * s * s)) * tof;

        // Solve using Battin's x iteration
        double x = solveForX(lambda, T, multi_rev);

        if (std::isnan(x))
        {
            sol.converged = false;
            return sol;
        }

        // Compute velocities
        double gamma = std::sqrt(mu * s / 2.0);
        double rho = (r1_mag - r2_mag) / c;
        double sigma = std::sqrt(1.0 - rho * rho);

        double y = std::sqrt(1.0 - lambda_sq * (1.0 - x * x));
        double vr1 = gamma * ((lambda * y - x) - rho * (lambda * y + x)) / r1_mag;
        double vr2 = gamma * ((lambda * y - x) + rho * (lambda * y + x)) / r2_mag;
        double vt1 = gamma * sigma * (y + lambda * x) / r1_mag;
        double vt2 = gamma * sigma * (y + lambda * x) / r2_mag;

        // Convert to vectors
        hpop_core::Vec3 r1_hat = r1.normalized();
        hpop_core::Vec3 r2_hat = r2.normalized();
        hpop_core::Vec3 h_hat = h.normalized();
        hpop_core::Vec3 t1_hat = h_hat.cross(r1_hat);
        hpop_core::Vec3 t2_hat = h_hat.cross(r2_hat);

        sol.v1 = r1_hat * vr1 + t1_hat * vt1;
        sol.v2 = r2_hat * vr2 + t2_hat * vt2;
        sol.converged = true;

        return sol;
    }

    /**
     * @brief Solve Lambert with initial and final velocities known
     *
     * Calculates delta-v required for rendezvous.
     *
     * @param state1 Initial state (position + velocity)
     * @param state2 Final state (position + velocity)
     * @param tof Time of flight [s]
     * @param mu Gravitational parameter
     * @return LambertSolution with delta-v computed
     */
    static LambertSolution solveRendezvous(const hpop_core::StateVector& state1,
                                           const hpop_core::StateVector& state2,
                                           double tof,
                                           double mu = hpop_core::constants::EARTH_MU)
    {
        LambertSolution sol = solve(state1.position, state2.position, tof, mu);

        if (sol.converged)
        {
            // Delta-v at departure
            hpop_core::Vec3 dv1 = sol.v1 - state1.velocity;
            sol.dv1 = dv1.norm();

            // Delta-v at arrival
            hpop_core::Vec3 dv2 = state2.velocity - sol.v2;
            sol.dv2 = dv2.norm();

            sol.total_dv = sol.dv1 + sol.dv2;
        }

        return sol;
    }

    /**
     * @brief Find optimal time of flight for minimum delta-v transfer
     *
     * @param r1 Initial position [m]
     * @param r2 Final position [m]
     * @param v1 Initial velocity [m/s]
     * @param v2 Final velocity [m/s]
     * @param tof_min Minimum time of flight [s]
     * @param tof_max Maximum time of flight [s]
     * @param steps Number of search steps
     * @return Optimal LambertSolution
     */
    static LambertSolution findOptimalTOF(const hpop_core::Vec3& r1,
                                          const hpop_core::Vec3& r2,
                                          const hpop_core::Vec3& v1,
                                          const hpop_core::Vec3& v2,
                                          double tof_min,
                                          double tof_max,
                                          int steps = 100)
    {
        LambertSolution best_sol;
        best_sol.total_dv = std::numeric_limits<double>::max();

        double dt = (tof_max - tof_min) / steps;

        for (int i = 0; i < steps; ++i)
        {
            double tof = tof_min + i * dt;
            LambertSolution sol = solve(r1, r2, tof);

            if (sol.converged)
            {
                hpop_core::Vec3 dv1 = sol.v1 - v1;
                hpop_core::Vec3 dv2 = v2 - sol.v2;
                sol.dv1 = dv1.norm();
                sol.dv2 = dv2.norm();
                sol.total_dv = sol.dv1 + sol.dv2;

                if (sol.total_dv < best_sol.total_dv)
                {
                    best_sol = sol;
                }
            }
        }

        return best_sol;
    }

private:
    /**
     * @brief Solve for x parameter using Halley iteration
     */
    static double solveForX(double lambda, double T, int multi_rev = 0)
    {
        // Initial guess
        double x;
        if (multi_rev == 0)
        {
            // Zero revolution case
            double T0 = std::acos(lambda) + lambda * std::sqrt(1.0 - lambda * lambda);
            double T1 = 2.0 * (1.0 - lambda * lambda * lambda) / 3.0;

            if (T < T0)
            {
                x = T0 / T - 1.0;
            }
            else
            {
                x = std::pow(T1 / T, 2.0 / 3.0) - 1.0;
            }
        }
        else
        {
            x = 0.0;
        }

        // Halley iteration
        constexpr int max_iter = 50;
        constexpr double tol = 1e-12;

        for (int iter = 0; iter < max_iter; ++iter)
        {
            double y = std::sqrt(1.0 - lambda * lambda * (1.0 - x * x));
            double fx = timeFunction(x, lambda, multi_rev) - T;
            double dfx = timeDerivative(x, lambda, y);
            double d2fx = timeSecondDerivative(x, lambda, y);

            // Halley step
            double delta = fx * dfx / (dfx * dfx - 0.5 * fx * d2fx);

            if (std::abs(delta) < tol)
            {
                return x;
            }

            x -= delta;

            // Bound x to valid range
            x = std::max(-1.0 + 1e-10, std::min(1.0 - 1e-10, x));
        }

        return std::nan("");  // Did not converge
    }

    /**
     * @brief Time of flight function
     */
    static double timeFunction(double x, double lambda, int multi_rev)
    {
        double y = std::sqrt(1.0 - lambda * lambda * (1.0 - x * x));

        double psi = computePsi(x, y, lambda);

        double T;
        if (std::abs(x) < 1.0)
        {
            T = (psi + multi_rev * hpop_core::constants::PI -
                 x * y * std::sqrt(1.0 - lambda * lambda)) /
                std::sqrt(1.0 - lambda * lambda);
        }
        else
        {
            T = 0.0;  // Hyperbolic case (simplified)
        }

        return T;
    }

    static double computePsi(double x, double y, double lambda)
    {
        double arg = lambda * y + x;
        if (arg > 1.0) arg = 1.0;
        if (arg < -1.0) arg = -1.0;
        return std::acos(arg);
    }

    /**
     * @brief Time function first derivative
     */
    static double timeDerivative(double x, double lambda, double y)
    {
        double sq_lam = lambda * lambda;
        double sq_x = x * x;

        if (std::abs(1.0 - sq_lam) < 1e-10)
        {
            return -2.0;
        }

        return (1.0 / (1.0 - sq_lam)) *
               (3.0 * x * y / (1.0 - sq_x) - 2.0 + 2.0 * sq_lam * x / y);
    }

    /**
     * @brief Time function second derivative
     */
    static double timeSecondDerivative(double x, double lambda, double y)
    {
        double sq_lam = lambda * lambda;
        double sq_x = x * x;
        double cu_x = sq_x * x;

        if (std::abs(1.0 - sq_lam) < 1e-10)
        {
            return 0.0;
        }

        return (1.0 / (1.0 - sq_lam)) *
               (3.0 * (1.0 + 2.0 * sq_x) * y / std::pow(1.0 - sq_x, 2) +
                2.0 * sq_lam / (y * y * y));
    }
};

/**
 * @brief Multi-revolution Lambert solver
 *
 * Finds all solutions for different numbers of revolutions.
 */
class MultiRevLambertSolver
{
public:
    /**
     * @brief Find all valid Lambert solutions
     *
     * @param r1 Initial position [m]
     * @param r2 Final position [m]
     * @param tof Time of flight [s]
     * @param max_revs Maximum revolutions to check
     * @return Vector of solutions sorted by delta-v
     */
    static std::vector<LambertSolution> findAll(const hpop_core::Vec3& r1,
                                                const hpop_core::Vec3& r2,
                                                double tof,
                                                int max_revs = 5)
    {
        std::vector<LambertSolution> solutions;

        // Zero revolution (short way and long way)
        for (bool prograde : {true, false})
        {
            auto sol = LambertSolver::solve(r1, r2, tof,
                                            hpop_core::constants::EARTH_MU,
                                            prograde, 0);
            if (sol.converged)
            {
                solutions.push_back(sol);
            }
        }

        // Multi-revolution solutions
        for (int n = 1; n <= max_revs; ++n)
        {
            for (bool prograde : {true, false})
            {
                auto sol = LambertSolver::solve(r1, r2, tof,
                                                hpop_core::constants::EARTH_MU,
                                                prograde, n);
                if (sol.converged)
                {
                    solutions.push_back(sol);
                }
            }
        }

        return solutions;
    }
};

} // namespace hpop_maneuver

#endif // HPOP_MANEUVER_LAMBERT_SOLVER_HPP
