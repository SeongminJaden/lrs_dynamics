#include "gazebo_leo_gravity/ggm_model.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <vector>
#include <omp.h>

using namespace gazebo_leo_gravity;

void testNmax(int nmax, const std::string& ggm_file)
{
    GGMModel model;

    auto t0 = std::chrono::high_resolution_clock::now();
    bool loaded = model.load(ggm_file, nmax);
    auto t1 = std::chrono::high_resolution_clock::now();

    if (!loaded)
    {
        std::cerr << "Failed to load GGM file for nmax=" << nmax << "\n";
        return;
    }

    auto load_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    // Test position: LEO 500km
    double R = 6378136.3;
    ignition::math::Vector3d test_pos(R + 500000, 0, 0);

    // Warmup
    for (int i = 0; i < 100; ++i)
        model.acceleration(test_pos);

    // Benchmark
    const int iterations = 10000;
    auto t2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
        volatile auto acc = model.acceleration(test_pos);
        (void)acc;
    }
    auto t3 = std::chrono::high_resolution_clock::now();

    auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    double per_call_us = static_cast<double>(total_us) / iterations;

    auto acc = model.acceleration(test_pos);

    std::cout << "| " << std::setw(4) << nmax
              << " | " << std::setw(6) << model.coeffCount()
              << " | " << std::setw(6) << load_ms << " ms"
              << " | " << std::setw(8) << std::fixed << std::setprecision(2) << per_call_us << " us"
              << " | " << std::setw(10) << std::setprecision(0) << (1000000.0 / per_call_us) << "/s"
              << " | " << std::setprecision(6) << acc.Length() << " m/s^2 |\n";
}

void testOpenMP(int nmax, const std::string& ggm_file)
{
    std::cout << "\n=== OpenMP Parallel Test (nmax=" << nmax << ") ===\n";

    int max_threads = omp_get_max_threads();
    std::cout << "Max threads: " << max_threads << "\n\n";

    // Create per-thread models
    std::vector<GGMModel> models(max_threads);
    for (int i = 0; i < max_threads; ++i)
        models[i].load(ggm_file, nmax);

    // Simulate multiple satellites
    double R = 6378136.3;
    std::vector<ignition::math::Vector3d> positions;
    for (int i = 0; i < 100; ++i)
    {
        double angle = i * 0.0628;  // ~3.6 degrees
        double alt = 400000 + (i % 10) * 10000;  // 400-490km
        positions.emplace_back(
            (R + alt) * std::cos(angle),
            (R + alt) * std::sin(angle),
            0
        );
    }

    const int n_sats = static_cast<int>(positions.size());
    const int iterations = 1000;

    std::cout << "| Threads | Time (ms) | Speedup |\n";
    std::cout << "|---------|-----------|----------|\n";

    double baseline = 0;

    for (int threads = 1; threads <= max_threads; threads *= 2)
    {
        omp_set_num_threads(threads);

        auto t0 = std::chrono::high_resolution_clock::now();

        for (int iter = 0; iter < iterations; ++iter)
        {
            #pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < n_sats; ++i)
            {
                int tid = omp_get_thread_num();
                volatile auto acc = models[tid].acceleration(positions[i]);
                (void)acc;
            }
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;

        if (threads == 1) baseline = ms;

        std::cout << "| " << std::setw(7) << threads
                  << " | " << std::setw(9) << std::fixed << std::setprecision(2) << ms
                  << " | " << std::setw(8) << std::setprecision(2) << (baseline / ms) << "x |\n";
    }
}

int main()
{
    std::string ggm_file = "/home/seongmin/ros2_ws/src/lrs_dynamics/data/GGM05C.gfc";

    std::cout << "=== GGM Model Performance Test ===\n\n";

    std::cout << "| nmax | Coeffs | Load    | Per Call  | Throughput | Gravity |\n";
    std::cout << "|------|--------|---------|-----------|------------|---------|\n";

    // Test different nmax values
    for (int nmax : {10, 20, 30, 50, 70, 100})
    {
        testNmax(nmax, ggm_file);
    }

    // OpenMP test
    testOpenMP(50, ggm_file);

    return 0;
}
