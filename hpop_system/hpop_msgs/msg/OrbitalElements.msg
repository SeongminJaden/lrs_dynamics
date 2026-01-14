# Classical Keplerian orbital elements

float64 semi_major_axis      # [m] Semi-major axis (a)
float64 eccentricity         # [-] Eccentricity (e), 0 <= e < 1 for elliptic
float64 inclination          # [rad] Inclination (i)
float64 raan                 # [rad] Right Ascension of Ascending Node (Omega)
float64 arg_periapsis        # [rad] Argument of Periapsis (omega)
float64 true_anomaly         # [rad] True Anomaly (nu)

# Alternative anomalies (optional)
float64 mean_anomaly         # [rad] Mean Anomaly (M)
float64 eccentric_anomaly    # [rad] Eccentric Anomaly (E)

# Derived quantities
float64 period               # [s] Orbital period
float64 mean_motion          # [rad/s] Mean motion (n)
float64 apoapsis             # [m] Apoapsis altitude from Earth center
float64 periapsis            # [m] Periapsis altitude from Earth center
