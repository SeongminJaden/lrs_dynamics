#!/usr/bin/env python3
"""
GGM05C Gravity Model Visualization - Individual Figures

Generates individual figures for each subplot for PPT use.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# Set publication-quality style
plt.rcParams.update({
    'font.size': 14,
    'font.family': 'serif',
    'axes.labelsize': 16,
    'axes.titlesize': 18,
    'legend.fontsize': 12,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# Paths
DATA_FILE = '/home/seongmin/ros2_ws/src/lrs_dynamics/data/ggm05c_sample_data.csv'
OUTPUT_DIR = '/home/seongmin/ros2_ws/src/lrs_dynamics/data/figures/individual'

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("Loading GGM05C sample data...")
    df = pd.read_csv(DATA_FILE)

    # Split data
    df_alt = df[(df['lat_deg'] == 0) & (df['lon_deg'] == 0)].copy()
    df_lat = df[(df['lon_deg'] == 0) & (df['alt_km'] == 400) &
                (df['lat_deg'].between(-90, 90))].drop_duplicates(subset=['lat_deg']).copy()
    df_grid = df[(df['alt_km'] == 400) & (df['lon_deg'] != 0)].copy()
    df_orbit = df[df['alt_km'] == 420].copy()

    # ==========================================
    # Figure 1a: Gravity vs Altitude
    # ==========================================
    print("Generating fig1a_gravity_vs_altitude...")
    fig, ax = plt.subplots(figsize=(10, 7))

    x_alt = df_alt['alt_km'].to_numpy()
    y_g_total = df_alt['g_total_m_s2'].to_numpy()
    y_g_point = df_alt['g_point_mass_m_s2'].to_numpy()

    ax.plot(x_alt, y_g_total, 'b-', linewidth=2.5, label='GGM05C (nmax=70)')
    ax.plot(x_alt, y_g_point, 'r--', linewidth=2.5, label='Point Mass')
    ax.set_xlabel('Altitude [km]')
    ax.set_ylabel('Gravitational Acceleration [m/s²]')
    ax.set_title('Gravity vs Altitude at Equator')
    ax.legend(loc='upper right')
    ax.set_xlim(200, 2000)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig1a_gravity_vs_altitude.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig1a_gravity_vs_altitude.pdf'))
    plt.close(fig)

    # ==========================================
    # Figure 1b: Gravity Anomaly vs Altitude
    # ==========================================
    print("Generating fig1b_anomaly_vs_altitude...")
    fig, ax = plt.subplots(figsize=(10, 7))

    y_delta = df_alt['delta_g_mGal'].to_numpy()

    ax.plot(x_alt, y_delta, 'g-', linewidth=2.5)
    ax.fill_between(x_alt, y_delta, alpha=0.3, color='green')
    ax.set_xlabel('Altitude [km]')
    ax.set_ylabel('Gravity Anomaly [mGal]')
    ax.set_title('GGM05C - Point Mass Difference at Equator')
    ax.set_xlim(200, 2000)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig1b_anomaly_vs_altitude.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig1b_anomaly_vs_altitude.pdf'))
    plt.close(fig)

    # ==========================================
    # Figure 2a: Gravity vs Latitude
    # ==========================================
    print("Generating fig2a_gravity_vs_latitude...")
    fig, ax = plt.subplots(figsize=(10, 7))

    df_lat_sorted = df_lat.sort_values('lat_deg')
    x_lat = df_lat_sorted['lat_deg'].to_numpy()
    y_g_lat = df_lat_sorted['g_total_m_s2'].to_numpy()
    y_g_point_lat = df_lat_sorted['g_point_mass_m_s2'].to_numpy()

    ax.plot(x_lat, y_g_lat, 'b-', linewidth=2.5, label='GGM05C')
    ax.plot(x_lat, y_g_point_lat, 'r--', linewidth=2.5, label='Point Mass')
    ax.set_xlabel('Latitude [deg]')
    ax.set_ylabel('Gravitational Acceleration [m/s²]')
    ax.set_title('Gravity vs Latitude at 400km Altitude')
    ax.legend(loc='upper right')
    ax.set_xlim(-90, 90)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig2a_gravity_vs_latitude.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig2a_gravity_vs_latitude.pdf'))
    plt.close(fig)

    # ==========================================
    # Figure 2b: Gravity Anomaly vs Latitude (J2 Effect)
    # ==========================================
    print("Generating fig2b_anomaly_vs_latitude...")
    fig, ax = plt.subplots(figsize=(10, 7))

    y_delta_lat = df_lat_sorted['delta_g_mGal'].to_numpy()

    ax.plot(x_lat, y_delta_lat, 'g-', linewidth=2.5)
    ax.fill_between(x_lat, y_delta_lat, alpha=0.3, color='green')
    ax.set_xlabel('Latitude [deg]')
    ax.set_ylabel('Gravity Anomaly [mGal]')
    ax.set_title('Latitude-dependent Gravity Anomaly (J2 Effect) at 400km')
    ax.set_xlim(-90, 90)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    # Add annotations
    ax.annotate(f'+{y_delta_lat[len(y_delta_lat)//2]:.0f} mGal',
                xy=(0, y_delta_lat[len(y_delta_lat)//2]),
                xytext=(20, y_delta_lat[len(y_delta_lat)//2]+200),
                fontsize=12, color='darkgreen',
                arrowprops=dict(arrowstyle='->', color='darkgreen'))
    ax.annotate(f'{y_delta_lat[0]:.0f} mGal',
                xy=(-90, y_delta_lat[0]),
                xytext=(-70, y_delta_lat[0]+200),
                fontsize=12, color='darkblue',
                arrowprops=dict(arrowstyle='->', color='darkblue'))

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig2b_anomaly_vs_latitude.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig2b_anomaly_vs_latitude.pdf'))
    plt.close(fig)

    # ==========================================
    # Figure 3: Global Gravity Anomaly Map
    # ==========================================
    print("Generating fig3_global_anomaly_map...")
    fig, ax = plt.subplots(figsize=(14, 8))

    df_map = pd.concat([df_grid, df_lat[df_lat['lon_deg'] == 0]]).drop_duplicates(
        subset=['lat_deg', 'lon_deg'])
    pivot = df_map.pivot_table(index='lat_deg', columns='lon_deg', values='delta_g_mGal')

    lons = pivot.columns.to_numpy()
    lats = pivot.index.to_numpy()
    LON, LAT = np.meshgrid(lons, lats)
    Z = pivot.to_numpy()

    ax.grid(False)
    c = ax.contourf(LON, LAT, Z, levels=50, cmap='RdYlBu_r')
    cbar = plt.colorbar(c, ax=ax, label='Gravity Anomaly [mGal]', shrink=0.8)

    ax.set_xlabel('Longitude [deg]')
    ax.set_ylabel('Latitude [deg]')
    ax.set_title('Global Gravity Anomaly at 400km Altitude (GGM05C, nmax=70)')
    ax.set_xlim(-180, 170)
    ax.set_ylim(-90, 90)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig3_global_anomaly_map.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig3_global_anomaly_map.pdf'))
    plt.close(fig)

    # ==========================================
    # Figure 4a: ISS Ground Track
    # ==========================================
    print("Generating fig4a_iss_ground_track...")
    if len(df_orbit) > 0:
        fig, ax = plt.subplots(figsize=(12, 7))

        df_orbit_sorted = df_orbit.reset_index(drop=True)
        lon_orbit = df_orbit_sorted['lon_deg'].to_numpy()
        lat_orbit = df_orbit_sorted['lat_deg'].to_numpy()
        delta_orbit = df_orbit_sorted['delta_g_mGal'].to_numpy()

        ax.grid(False)
        sc = ax.scatter(lon_orbit, lat_orbit, c=delta_orbit, cmap='RdYlBu_r', s=30)
        plt.colorbar(sc, ax=ax, label='Gravity Anomaly [mGal]', shrink=0.8)
        ax.set_xlabel('Longitude [deg]')
        ax.set_ylabel('Latitude [deg]')
        ax.set_title('ISS-like Orbit Ground Track (420km, 51.6° inclination)')
        ax.set_xlim(-180, 180)
        ax.set_ylim(-60, 60)

        plt.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4a_iss_ground_track.png'))
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4a_iss_ground_track.pdf'))
        plt.close(fig)

    # ==========================================
    # Figure 4b: Gravity Along Orbit
    # ==========================================
    print("Generating fig4b_gravity_along_orbit...")
    if len(df_orbit) > 0:
        fig, ax = plt.subplots(figsize=(10, 7))

        orbit_angle = np.arange(len(df_orbit_sorted))
        g_orbit = df_orbit_sorted['g_total_m_s2'].to_numpy()

        ax.plot(orbit_angle, g_orbit, 'b-', linewidth=2)
        ax.fill_between(orbit_angle, g_orbit.min(), g_orbit, alpha=0.2)
        ax.set_xlabel('Orbital Position [deg]')
        ax.set_ylabel('Gravitational Acceleration [m/s²]')
        ax.set_title('Gravity Along ISS Orbit (420km, 51.6° inc)')
        ax.set_xlim(0, 360)

        # Add annotations for max/min
        max_idx = np.argmax(g_orbit)
        min_idx = np.argmin(g_orbit)
        ax.annotate(f'Max: {g_orbit[max_idx]:.4f}', xy=(max_idx, g_orbit[max_idx]),
                   xytext=(max_idx+30, g_orbit[max_idx]+0.005), fontsize=11,
                   arrowprops=dict(arrowstyle='->', color='red'))
        ax.annotate(f'Min: {g_orbit[min_idx]:.4f}', xy=(min_idx, g_orbit[min_idx]),
                   xytext=(min_idx+30, g_orbit[min_idx]-0.005), fontsize=11,
                   arrowprops=dict(arrowstyle='->', color='blue'))

        plt.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4b_gravity_along_orbit.png'))
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4b_gravity_along_orbit.pdf'))
        plt.close(fig)

    # ==========================================
    # Figure 4c: Gravity Anomaly Along Orbit
    # ==========================================
    print("Generating fig4c_anomaly_along_orbit...")
    if len(df_orbit) > 0:
        fig, ax = plt.subplots(figsize=(10, 7))

        delta_orbit = df_orbit_sorted['delta_g_mGal'].to_numpy()

        ax.plot(orbit_angle, delta_orbit, 'g-', linewidth=2)
        ax.fill_between(orbit_angle, 0, delta_orbit, where=(delta_orbit >= 0),
                       alpha=0.3, color='red', label='Positive anomaly')
        ax.fill_between(orbit_angle, 0, delta_orbit, where=(delta_orbit < 0),
                       alpha=0.3, color='blue', label='Negative anomaly')
        ax.set_xlabel('Orbital Position [deg]')
        ax.set_ylabel('Gravity Anomaly [mGal]')
        ax.set_title('Gravity Anomaly Along ISS Orbit')
        ax.set_xlim(0, 360)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=1)
        ax.legend(loc='upper right')

        # Add range annotation
        ax.annotate(f'Range: {delta_orbit.max()-delta_orbit.min():.0f} mGal',
                   xy=(180, 0), xytext=(200, 300), fontsize=12,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4c_anomaly_along_orbit.png'))
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4c_anomaly_along_orbit.pdf'))
        plt.close(fig)

    # ==========================================
    # Figure 4d: Anomaly vs Latitude (orbit)
    # ==========================================
    print("Generating fig4d_anomaly_vs_latitude_orbit...")
    if len(df_orbit) > 0:
        fig, ax = plt.subplots(figsize=(10, 7))

        lat_orbit = df_orbit_sorted['lat_deg'].to_numpy()

        sc = ax.scatter(lat_orbit, delta_orbit, c=orbit_angle, cmap='viridis', s=30, alpha=0.8)
        plt.colorbar(sc, ax=ax, label='Orbital Position [deg]', shrink=0.8)
        ax.set_xlabel('Latitude [deg]')
        ax.set_ylabel('Gravity Anomaly [mGal]')
        ax.set_title('Gravity Anomaly vs Latitude Along Orbit')
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

        # Fit quadratic curve
        coeffs = np.polyfit(lat_orbit, delta_orbit, 2)
        lat_fit = np.linspace(-52, 52, 100)
        delta_fit = np.polyval(coeffs, lat_fit)
        ax.plot(lat_fit, delta_fit, 'r--', linewidth=2, label='Quadratic fit (J2)')
        ax.legend(loc='upper right')

        plt.tight_layout()
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4d_anomaly_vs_latitude_orbit.png'))
        fig.savefig(os.path.join(OUTPUT_DIR, 'fig4d_anomaly_vs_latitude_orbit.pdf'))
        plt.close(fig)

    # ==========================================
    # Figure 5: J2 Comparison
    # ==========================================
    print("Generating fig5_j2_comparison...")
    fig, ax = plt.subplots(figsize=(10, 7))

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

    ax.plot(x_lat, y_delta_lat, 'b-', linewidth=2.5, label='GGM05C (nmax=70)')
    ax.plot(x_lat, g_j2_mGal, 'r--', linewidth=2.5, label='J2-only (analytical)')
    ax.fill_between(x_lat, y_delta_lat, g_j2_mGal, alpha=0.3, color='green',
                   label='Higher-order terms contribution')

    ax.set_xlabel('Latitude [deg]')
    ax.set_ylabel('Gravity Anomaly [mGal]')
    ax.set_title('GGM05C vs J2-only Model Comparison at 400km')
    ax.legend(loc='upper right')
    ax.set_xlim(-90, 90)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig5_j2_comparison.png'))
    fig.savefig(os.path.join(OUTPUT_DIR, 'fig5_j2_comparison.pdf'))
    plt.close(fig)

    # ==========================================
    # Summary
    # ==========================================
    print("\n" + "="*60)
    print("Individual figures generated!")
    print("="*60)
    print(f"\nOutput directory: {OUTPUT_DIR}\n")

    for f in sorted(os.listdir(OUTPUT_DIR)):
        if f.endswith('.png'):
            fpath = os.path.join(OUTPUT_DIR, f)
            fsize = os.path.getsize(fpath) / 1024
            print(f"  {f} ({fsize:.1f} KB)")

    print("\n" + "="*60)

if __name__ == '__main__':
    main()
