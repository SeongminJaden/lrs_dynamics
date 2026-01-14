#!/usr/bin/env python3
"""
GGM05C Gravity Model Visualization

Generates publication-quality figures from GGM05C sample data.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import os

# Set publication-quality style
plt.rcParams.update({
    'font.size': 12,
    'font.family': 'serif',
    'axes.labelsize': 14,
    'axes.titlesize': 16,
    'legend.fontsize': 11,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'figure.figsize': (10, 6),
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# Paths
DATA_FILE = '/home/seongmin/ros2_ws/src/lrs_dynamics/data/ggm05c_sample_data.csv'
OUTPUT_DIR = '/home/seongmin/ros2_ws/src/lrs_dynamics/data/figures'

def main():
    # Create output directory
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Load data
    print("Loading GGM05C sample data...")
    df = pd.read_csv(DATA_FILE)
    print(f"Loaded {len(df)} data points")

    # Split data by type
    df_alt = df[(df['lat_deg'] == 0) & (df['lon_deg'] == 0)].copy()
    df_lat = df[(df['lon_deg'] == 0) & (df['alt_km'] == 400) &
                (df['lat_deg'].between(-90, 90))].drop_duplicates(subset=['lat_deg']).copy()
    df_grid = df[(df['alt_km'] == 400) & (df['lon_deg'] != 0)].copy()
    df_orbit = df[df['alt_km'] == 420].copy()

    print(f"  Altitude sweep: {len(df_alt)} points")
    print(f"  Latitude sweep: {len(df_lat)} points")
    print(f"  Global grid: {len(df_grid)} points")
    print(f"  LEO orbit: {len(df_orbit)} points")

    # ==========================================
    # Figure 1: Gravity vs Altitude
    # ==========================================
    print("\nGenerating Figure 1: Gravity vs Altitude...")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    x_alt = df_alt['alt_km'].to_numpy()
    y_g_total = df_alt['g_total_m_s2'].to_numpy()
    y_g_point = df_alt['g_point_mass_m_s2'].to_numpy()
    y_delta = df_alt['delta_g_mGal'].to_numpy()

    ax1 = axes[0]
    ax1.plot(x_alt, y_g_total, 'b-', linewidth=2, label='GGM05C (nmax=70)')
    ax1.plot(x_alt, y_g_point, 'r--', linewidth=2, label='Point Mass')
    ax1.set_xlabel('Altitude [km]')
    ax1.set_ylabel('Gravitational Acceleration [m/s²]')
    ax1.set_title('(a) Gravity vs Altitude at Equator')
    ax1.legend()
    ax1.set_xlim(200, 2000)

    ax2 = axes[1]
    ax2.plot(x_alt, y_delta, 'g-', linewidth=2)
    ax2.set_xlabel('Altitude [km]')
    ax2.set_ylabel('Gravity Anomaly [mGal]')
    ax2.set_title('(b) GGM05C - Point Mass Difference')
    ax2.set_xlim(200, 2000)
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_gravity_vs_altitude.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_gravity_vs_altitude.pdf'))
    print(f"  Saved: ggm05c_gravity_vs_altitude.png/pdf")
    plt.close(fig)

    # ==========================================
    # Figure 2: Gravity vs Latitude
    # ==========================================
    print("Generating Figure 2: Gravity vs Latitude...")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    df_lat_sorted = df_lat.sort_values('lat_deg')
    x_lat = df_lat_sorted['lat_deg'].to_numpy()
    y_g_lat = df_lat_sorted['g_total_m_s2'].to_numpy()
    y_g_point_lat = df_lat_sorted['g_point_mass_m_s2'].to_numpy()
    y_delta_lat = df_lat_sorted['delta_g_mGal'].to_numpy()

    ax1 = axes[0]
    ax1.plot(x_lat, y_g_lat, 'b-', linewidth=2, label='GGM05C')
    ax1.plot(x_lat, y_g_point_lat, 'r--', linewidth=2, label='Point Mass')
    ax1.set_xlabel('Latitude [deg]')
    ax1.set_ylabel('Gravitational Acceleration [m/s²]')
    ax1.set_title('(a) Gravity vs Latitude at 400km')
    ax1.legend()
    ax1.set_xlim(-90, 90)

    ax2 = axes[1]
    ax2.plot(x_lat, y_delta_lat, 'g-', linewidth=2)
    ax2.fill_between(x_lat, y_delta_lat, alpha=0.3)
    ax2.set_xlabel('Latitude [deg]')
    ax2.set_ylabel('Gravity Anomaly [mGal]')
    ax2.set_title('(b) Latitude-dependent Gravity Anomaly (J2 effect)')
    ax2.set_xlim(-90, 90)
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_gravity_vs_latitude.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_gravity_vs_latitude.pdf'))
    print(f"  Saved: ggm05c_gravity_vs_latitude.png/pdf")
    plt.close(fig)

    # ==========================================
    # Figure 3: Global Gravity Anomaly Map
    # ==========================================
    print("Generating Figure 3: Global Gravity Anomaly Map...")

    df_map = pd.concat([df_grid, df_lat[df_lat['lon_deg'] == 0]]).drop_duplicates(
        subset=['lat_deg', 'lon_deg'])

    pivot = df_map.pivot_table(index='lat_deg', columns='lon_deg', values='delta_g_mGal')

    fig, ax = plt.subplots(figsize=(14, 7))

    lons = pivot.columns.to_numpy()
    lats = pivot.index.to_numpy()
    LON, LAT = np.meshgrid(lons, lats)
    Z = pivot.to_numpy()

    c = ax.contourf(LON, LAT, Z, levels=50, cmap='RdYlBu_r')
    cbar = plt.colorbar(c, ax=ax, label='Gravity Anomaly [mGal]')

    ax.set_xlabel('Longitude [deg]')
    ax.set_ylabel('Latitude [deg]')
    ax.set_title('GGM05C Gravity Anomaly at 400km Altitude (nmax=70)')
    ax.set_xlim(-180, 170)
    ax.set_ylim(-90, 90)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax.axvline(x=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_global_anomaly_map.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_global_anomaly_map.pdf'))
    print(f"  Saved: ggm05c_global_anomaly_map.png/pdf")
    plt.close(fig)

    # ==========================================
    # Figure 4: LEO Orbit Gravity Variation
    # ==========================================
    print("Generating Figure 4: LEO Orbit Gravity Variation...")

    if len(df_orbit) > 0:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        df_orbit_sorted = df_orbit.reset_index(drop=True)
        orbit_angle = np.arange(len(df_orbit_sorted))

        lon_orbit = df_orbit_sorted['lon_deg'].to_numpy()
        lat_orbit = df_orbit_sorted['lat_deg'].to_numpy()
        delta_orbit = df_orbit_sorted['delta_g_mGal'].to_numpy()
        g_orbit = df_orbit_sorted['g_total_m_s2'].to_numpy()

        ax1 = axes[0, 0]
        sc = ax1.scatter(lon_orbit, lat_orbit, c=delta_orbit, cmap='RdYlBu_r', s=10)
        plt.colorbar(sc, ax=ax1, label='Anomaly [mGal]')
        ax1.set_xlabel('Longitude [deg]')
        ax1.set_ylabel('Latitude [deg]')
        ax1.set_title('(a) ISS-like Orbit Ground Track')
        ax1.set_xlim(-180, 180)
        ax1.set_ylim(-60, 60)

        ax2 = axes[0, 1]
        ax2.plot(orbit_angle, g_orbit, 'b-', linewidth=1.5)
        ax2.set_xlabel('Orbital Position [deg]')
        ax2.set_ylabel('Gravity [m/s²]')
        ax2.set_title('(b) Gravity Along ISS Orbit (420km, 51.6° inc)')
        ax2.set_xlim(0, 360)

        ax3 = axes[1, 0]
        ax3.plot(orbit_angle, delta_orbit, 'g-', linewidth=1.5)
        ax3.fill_between(orbit_angle, delta_orbit, alpha=0.3)
        ax3.set_xlabel('Orbital Position [deg]')
        ax3.set_ylabel('Gravity Anomaly [mGal]')
        ax3.set_title('(c) Gravity Anomaly Along Orbit')
        ax3.set_xlim(0, 360)
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

        ax4 = axes[1, 1]
        ax4.scatter(lat_orbit, delta_orbit, c=orbit_angle, cmap='viridis', s=20, alpha=0.7)
        ax4.set_xlabel('Latitude [deg]')
        ax4.set_ylabel('Gravity Anomaly [mGal]')
        ax4.set_title('(d) Anomaly vs Latitude (colored by orbital position)')

        plt.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_leo_orbit_gravity.png'))
        fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_leo_orbit_gravity.pdf'))
        print(f"  Saved: ggm05c_leo_orbit_gravity.png/pdf")
        plt.close(fig)

    # ==========================================
    # Figure 5: J2 Effect Comparison
    # ==========================================
    print("Generating Figure 5: Spherical Harmonics Effect...")

    fig, ax = plt.subplots(figsize=(10, 6))

    J2 = 1.08263e-3
    R_E = 6378136.3
    GM = 3.986004415e14

    df_lat_sorted = df_lat.sort_values('lat_deg').copy()
    r = (R_E + 400000)
    lat_rad = np.deg2rad(df_lat_sorted['lat_deg'].to_numpy())

    g_j2_approx = (3/2) * J2 * (R_E/r)**2 * GM / r**2 * (3*np.sin(lat_rad)**2 - 1)
    g_j2_mGal = g_j2_approx * 1e5

    x_lat = df_lat_sorted['lat_deg'].to_numpy()
    y_delta_lat = df_lat_sorted['delta_g_mGal'].to_numpy()

    ax.plot(x_lat, y_delta_lat, 'b-', linewidth=2, label='GGM05C (nmax=70)')
    ax.plot(x_lat, g_j2_mGal, 'r--', linewidth=2, label='J2-only (analytical)')
    ax.fill_between(x_lat, y_delta_lat, g_j2_mGal, alpha=0.3, label='Higher-order terms')

    ax.set_xlabel('Latitude [deg]')
    ax.set_ylabel('Gravity Anomaly [mGal]')
    ax.set_title('GGM05C vs J2-only Model at 400km Altitude')
    ax.legend()
    ax.set_xlim(-90, 90)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_j2_comparison.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'ggm05c_j2_comparison.pdf'))
    print(f"  Saved: ggm05c_j2_comparison.png/pdf")
    plt.close(fig)

    # ==========================================
    # Summary Statistics
    # ==========================================
    print("\n" + "="*60)
    print("GGM05C GRAVITY MODEL ANALYSIS SUMMARY")
    print("="*60)
    print(f"\nData points: {len(df)}")
    print(f"Model: GGM05C (nmax=70, 2556 coefficients)")

    print("\n--- Gravity Anomaly Statistics ---")
    print(f"  Min:  {df['delta_g_mGal'].min():.2f} mGal")
    print(f"  Max:  {df['delta_g_mGal'].max():.2f} mGal")
    print(f"  Mean: {df['delta_g_mGal'].mean():.2f} mGal")
    print(f"  Std:  {df['delta_g_mGal'].std():.2f} mGal")

    print("\n--- Output Files ---")
    for f in sorted(os.listdir(OUTPUT_DIR)):
        fpath = os.path.join(OUTPUT_DIR, f)
        fsize = os.path.getsize(fpath) / 1024
        print(f"  {f} ({fsize:.1f} KB)")

    print("\n" + "="*60)
    print("Visualization complete!")
    print("="*60)

if __name__ == '__main__':
    main()
