#include "gazebo_leo_gravity/ggm_model.hpp"
#include "gazebo_leo_gravity/legendre.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace gazebo_leo_gravity
{

bool GGMModel::load(const std::string& filename, int nmax)
{
    nmax_ = nmax;
    coeffs_.clear();

    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cerr << "[GGMModel] Warning: Failed to open file: " << filename << std::endl;
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
                std::stringstream ss(line);
                std::string key;
                ss >> key >> GM_;
            }
            else if (line.find("radius") != std::string::npos)
            {
                std::stringstream ss(line);
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
            for (auto &c : line) if (c == 'D') c = 'E';

            std::istringstream iss(line);
            std::string tag;
            int n, m;
            double C, S, sigmaC, sigmaS;
            iss >> tag >> n >> m >> C >> S >> sigmaC >> sigmaS;

            if (n <= nmax_)
                coeffs_.push_back({n, m, C, S});
        }
    }

    if (GM_ == 0) GM_ = 3.986004415e14;    // [m^3/s^2]
    if (a_ == 0)  a_ = 6378136.3;          // [m]

    std::cout << "[GGMModel] Loaded GGM file: " << filename
              << " (nmax=" << nmax_ << ", coeffs=" << coeffs_.size() << ")\n";

    return true;
}

ignition::math::Vector3d GGMModel::acceleration(const ignition::math::Vector3d& pos) const
{
    double x = pos.X();
    double y = pos.Y();
    double z = pos.Z();

    double r = std::sqrt(x*x + y*y + z*z);
    if (r == 0.0) return ignition::math::Vector3d::Zero;

    double phi = std::asin(z / r);               
    double lambda = std::atan2(y, x);            

    double cosphi = std::cos(phi);
    double sinphi = std::sin(phi);

    LegendreTable P(nmax_, sinphi, normalized_);

    double dUdr = 0.0;
    double dUdphi = 0.0;
    double dUdlambda = 0.0;

    for (auto &c : coeffs_)
    {
        int n = c.n;
        int m = c.m;
        double C = c.C;
        double S = c.S;

        double cosm = std::cos(m * lambda);
        double sinm = std::sin(m * lambda);

        double Pnm = P.get(n, m);

        double factor = std::pow(a_ / r, n);

        dUdr += (n+1) * factor * Pnm * (C * cosm + S * sinm);
        dUdphi += factor * P.dPhi(n, m, cosphi, sinphi) * (C * cosm + S * sinm);
        dUdlambda += factor * m * Pnm * (-C * sinm + S * cosm);
    }

    double common = GM_ / (r * r);

    double ar = -common * dUdr;
    double aphi = common * dUdphi;
    double alam = common * dUdlambda;

    double ax = (ar * cosphi * cos(lambda) - aphi * sinphi * cos(lambda) - alam * sin(lambda));
    double ay = (ar * cosphi * sin(lambda) - aphi * sinphi * sin(lambda) + alam * cos(lambda));
    double az = (ar * sinphi + aphi * cosphi);

    return ignition::math::Vector3d(ax, ay, az);
}

} // namespace gazebo_leo_gravity

