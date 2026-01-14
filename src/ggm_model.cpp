#include "gazebo_leo_gravity/ggm_model.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace gazebo_leo_gravity
{

bool GGMModel::load(const std::string& filename, int nmax)
{
    nmax_ = nmax;
    coeffs_.clear();

    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cerr << "[GGMModel] Failed to open file: " << filename << std::endl;
        return false;
    }

    std::string line;
    bool inHeader = true;

    while (std::getline(fin, line))
    {
        if (line.find("end_of_head") != std::string::npos)
        {
            inHeader = false;
            continue;
        }

        if (inHeader)
        {
            if (line.find("earth_gravity_constant") != std::string::npos)
            {
                std::istringstream ss(line);
                std::string key;
                ss >> key >> GM_;
            }
            else if (line.find("radius") != std::string::npos)
            {
                std::istringstream ss(line);
                std::string key;
                ss >> key >> a_;
            }
            else if (line.find("norm") != std::string::npos)
            {
                normalized_ = (line.find("fully_normalized") != std::string::npos);
            }
            continue;
        }

        if (line.rfind("gfc", 0) == 0)
        {
            std::replace(line.begin(), line.end(), 'D', 'E');

            std::istringstream iss(line);
            std::string tag;
            int n, m;
            double C, S, sigmaC, sigmaS;
            iss >> tag >> n >> m >> C >> S >> sigmaC >> sigmaS;

            if (n <= nmax_)
                coeffs_.push_back({n, m, C, S});
        }
    }

    if (GM_ == 0) GM_ = 3.986004415e14;
    if (a_ == 0)  a_ = 6378136.3;

    // Sort by degree for cache locality
    std::sort(coeffs_.begin(), coeffs_.end(),
        [](const Coefficient& a, const Coefficient& b) {
            return (a.n != b.n) ? (a.n < b.n) : (a.m < b.m);
        });

    // Pre-allocate buffers
    ensureBuffers();

    std::cout << "[GGMModel] Loaded: " << filename
              << " (nmax=" << nmax_ << ", coeffs=" << coeffs_.size() << ")\n";

    return true;
}

void GGMModel::ensureBuffers() const
{
    if (static_cast<int>(a_r_pow_.size()) < nmax_ + 2)
    {
        a_r_pow_.resize(nmax_ + 2);
        cosm_.resize(nmax_ + 1);
        sinm_.resize(nmax_ + 1);

        P_.resize(nmax_ + 1);
        for (int n = 0; n <= nmax_; ++n)
            P_[n].resize(n + 1, 0.0);
    }
}

void GGMModel::computeLegendre(double sinphi, double cosphi) const
{
    // Base case
    P_[0][0] = 1.0;
    if (nmax_ == 0) return;

    // Compute unnormalized Legendre polynomials
    for (int n = 1; n <= nmax_; ++n)
    {
        // Diagonal: Pnn
        P_[n][n] = (2*n - 1) * cosphi * P_[n-1][n-1];

        // Sub-diagonal: Pn,n-1
        P_[n][n-1] = (2*n - 1) * sinphi * P_[n-1][n-1];

        // Tesseral terms
        for (int m = 0; m <= n - 2; ++m)
        {
            const double Pn1m = P_[n-1][m];
            const double Pn2m = (n >= 2 && m <= n-2) ? P_[n-2][m] : 0.0;
            P_[n][m] = ((2*n - 1) * sinphi * Pn1m - (n + m - 1) * Pn2m) / (n - m);
        }
    }

    // Apply normalization
    if (normalized_)
    {
        for (int n = 0; n <= nmax_; ++n)
        {
            for (int m = 0; m <= n; ++m)
            {
                double ratio = 1.0;
                for (int k = n - m + 1; k <= n + m; ++k)
                    ratio *= k;

                const double delta = (m == 0) ? 1.0 : 0.0;
                P_[n][m] *= std::sqrt((2.0 - delta) / ratio);
            }
        }
    }
}

ignition::math::Vector3d GGMModel::acceleration(const ignition::math::Vector3d& pos) const
{
    const double x = pos.X();
    const double y = pos.Y();
    const double z = pos.Z();

    const double r2 = x*x + y*y + z*z;
    if (r2 < 1.0) return ignition::math::Vector3d::Zero;

    const double r = std::sqrt(r2);
    const double r_inv = 1.0 / r;

    const double sinphi = z * r_inv;
    const double cosphi = std::sqrt(1.0 - sinphi * sinphi);
    const double lambda = std::atan2(y, x);
    const double coslam = std::cos(lambda);
    const double sinlam = std::sin(lambda);

    // Precompute (a/r)^n using incremental multiplication
    const double a_over_r = a_ * r_inv;
    a_r_pow_[0] = 1.0;
    for (int n = 1; n <= nmax_ + 1; ++n)
        a_r_pow_[n] = a_r_pow_[n-1] * a_over_r;

    // Precompute cos(m*lambda), sin(m*lambda) using recurrence
    cosm_[0] = 1.0;
    sinm_[0] = 0.0;
    for (int m = 1; m <= nmax_; ++m)
    {
        cosm_[m] = cosm_[m-1] * coslam - sinm_[m-1] * sinlam;
        sinm_[m] = sinm_[m-1] * coslam + cosm_[m-1] * sinlam;
    }

    // Compute Legendre polynomials in-place
    computeLegendre(sinphi, cosphi);

    double dUdr = 0.0;
    double dUdphi = 0.0;
    double dUdlambda = 0.0;

    const double cosphi_inv = (std::abs(cosphi) > 1e-15) ? (1.0 / cosphi) : 0.0;

    for (const auto& c : coeffs_)
    {
        const int n = c.n;
        const int m = c.m;

        const double Pnm = P_[n][m];
        const double factor = a_r_pow_[n];

        const double CS_cosm = c.C * cosm_[m] + c.S * sinm_[m];
        const double CS_sinm = -c.C * sinm_[m] + c.S * cosm_[m];

        dUdr += (n + 1) * factor * Pnm * CS_cosm;

        // Compute dP/dphi inline
        double dPnm = 0.0;
        if (n > 0 && cosphi_inv != 0.0)
        {
            if (n == m)
            {
                dPnm = -n * sinphi * cosphi_inv * Pnm;
            }
            else
            {
                const double Pn1m = (n > 0 && m <= n-1) ? P_[n-1][m] : 0.0;
                dPnm = (n * sinphi * Pnm - (n + m) * Pn1m) * cosphi_inv;
            }
        }

        dUdphi += factor * dPnm * CS_cosm;
        dUdlambda += factor * m * Pnm * CS_sinm;
    }

    const double common = GM_ / r2;

    const double ar   = -common * dUdr;
    const double aphi =  common * dUdphi;
    const double alam =  common * dUdlambda;

    // Spherical to Cartesian transformation
    const double ax = ar * cosphi * coslam - aphi * sinphi * coslam - alam * sinlam;
    const double ay = ar * cosphi * sinlam - aphi * sinphi * sinlam + alam * coslam;
    const double az = ar * sinphi + aphi * cosphi;

    return ignition::math::Vector3d(ax, ay, az);
}

} // namespace gazebo_leo_gravity
