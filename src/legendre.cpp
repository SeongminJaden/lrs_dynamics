#include "gazebo_leo_gravity/legendre.hpp"
#include <stdexcept>
#include <array>

namespace gazebo_leo_gravity
{

// Pre-computed factorial table (up to 360! would overflow, use log-factorial for normalization)
namespace {
    constexpr int MAX_FACTORIAL = 170;  // Max before double overflow

    // Compute factorial table at compile time
    constexpr std::array<double, MAX_FACTORIAL + 1> computeFactorialTable() {
        std::array<double, MAX_FACTORIAL + 1> table{};
        table[0] = 1.0;
        for (int i = 1; i <= MAX_FACTORIAL; ++i) {
            table[i] = table[i-1] * i;
        }
        return table;
    }

    constexpr auto FACTORIAL_TABLE = computeFactorialTable();
}

// --------------------------------------
// Factorial utility with caching
// --------------------------------------
double LegendreTable::factorial(int n)
{
    if (n <= 1) return 1.0;
    if (n <= MAX_FACTORIAL) return FACTORIAL_TABLE[n];

    // For very large n, compute iteratively (shouldn't happen with typical nmax)
    double f = FACTORIAL_TABLE[MAX_FACTORIAL];
    for (int i = MAX_FACTORIAL + 1; i <= n; ++i) f *= i;
    return f;
}

// --------------------------------------
// Constructor: compute fully-normalized Pnm
// Using stable recurrence relations
// --------------------------------------
LegendreTable::LegendreTable(int nmax, double sinphi, bool normalized)
    : nmax_(nmax), normalized_(normalized)
{
    const double cosphi = std::sqrt(1.0 - sinphi * sinphi);

    P_.resize(nmax_ + 1);
    for (int n = 0; n <= nmax_; ++n)
        P_[n].resize(n + 1, 0.0);

    // Base case P0,0
    P_[0][0] = 1.0;

    if (nmax_ == 0) return;

    // Use standard recurrence for unnormalized, then apply normalization
    for (int n = 1; n <= nmax_; ++n)
    {
        // Sectoral term Pnn (diagonal)
        P_[n][n] = (2*n - 1) * cosphi * P_[n-1][n-1];

        // Sub-diagonal term Pn,n-1
        if (n >= 1)
            P_[n][n-1] = (2*n - 1) * sinphi * P_[n-1][n-1];

        // Tesseral terms (m < n-1)
        for (int m = 0; m <= n - 2; ++m)
        {
            double Pn1m = (n - 1 >= m) ? P_[n-1][m] : 0.0;
            double Pn2m = (n - 2 >= m) ? P_[n-2][m] : 0.0;
            P_[n][m] = ((2*n - 1) * sinphi * Pn1m - (n + m - 1) * Pn2m) / (n - m);
        }
    }

    // Apply full normalization if required
    if (normalized_)
    {
        for (int n = 0; n <= nmax_; ++n)
        {
            for (int m = 0; m <= n; ++m)
            {
                // Normalization factor: sqrt((2 - delta_m0) * (n-m)! / (n+m)!)
                // Use ratio computation to avoid overflow
                double ratio = 1.0;
                for (int k = n - m + 1; k <= n + m; ++k)
                    ratio *= k;

                double delta = (m == 0) ? 1.0 : 0.0;
                double factor = std::sqrt((2.0 - delta) / ratio);
                P_[n][m] *= factor;
            }
        }
    }
}

// --------------------------------------
// Return Pnm value
// --------------------------------------
double LegendreTable::get(int n, int m) const
{
    if (n < 0 || n > nmax_ || m < 0 || m > n)
        throw std::out_of_range("Invalid n or m");
    return P_[n][m];
}

// --------------------------------------
// Compute derivative w.r.t φ using recurrence
// dPnm/dφ = (n * sinφ * Pnm - (n+m) * P_{n-1,m}) / cosφ
// --------------------------------------
double LegendreTable::dPhi(int n, int m, double cosphi, double sinphi) const
{
    if (m > n || n == 0) return 0.0;

    // For m == n (diagonal), use: dPnn/dφ = -n * tanφ * Pnn
    if (n == m) {
        if (std::abs(cosphi) < 1e-15) return 0.0;
        return -n * (sinphi / cosphi) * P_[n][m];
    }

    // General case
    if (std::abs(cosphi) < 1e-15) return 0.0;

    double Pn1m = (n - 1 >= m) ? P_[n-1][m] : 0.0;
    return (n * sinphi * P_[n][m] - (n + m) * Pn1m) / cosphi;
}

} // namespace gazebo_leo_gravity
