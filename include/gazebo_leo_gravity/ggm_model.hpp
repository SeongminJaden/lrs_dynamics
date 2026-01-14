#ifndef GGM_MODEL_HPP_
#define GGM_MODEL_HPP_

#include <string>
#include <vector>
#include <ignition/math/Vector3.hh>

namespace gazebo_leo_gravity
{

struct Coefficient
{
    int n, m;
    double C, S;
};

class GGMModel
{
public:
    GGMModel() = default;

    // Load coefficients from file
    bool load(const std::string& filename, int nmax);

    // Compute gravitational acceleration at position (ECEF coordinates)
    ignition::math::Vector3d acceleration(const ignition::math::Vector3d& pos) const;

    // Getters
    int nmax() const { return nmax_; }
    double GM() const { return GM_; }
    double radius() const { return a_; }
    size_t coeffCount() const { return coeffs_.size(); }

private:
    int nmax_ = 0;
    double GM_ = 3.986004415e14;   // [m^3/s^2]
    double a_ = 6378136.3;          // [m]
    bool normalized_ = true;

    std::vector<Coefficient> coeffs_;

    // Pre-allocated buffers for acceleration computation (mutable for const method)
    mutable std::vector<double> a_r_pow_;
    mutable std::vector<double> cosm_;
    mutable std::vector<double> sinm_;
    mutable std::vector<std::vector<double>> P_;  // Legendre table cache

    // Internal methods
    void computeLegendre(double sinphi, double cosphi) const;
    void ensureBuffers() const;
};

} // namespace gazebo_leo_gravity

#endif  // GGM_MODEL_HPP_
