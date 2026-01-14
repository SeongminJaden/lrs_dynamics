# Acceleration components from various perturbation sources

std_msgs/Header header
string satellite_id

# Total acceleration [m/s^2]
geometry_msgs/Vector3 total

# Individual perturbation contributions [m/s^2]
geometry_msgs/Vector3 gravity_central    # Central body (point mass)
geometry_msgs/Vector3 gravity_harmonics  # Spherical harmonics (J2, J3, ..., Jn)
geometry_msgs/Vector3 atmospheric_drag   # Atmospheric drag
geometry_msgs/Vector3 solar_radiation    # Solar radiation pressure
geometry_msgs/Vector3 third_body_moon    # Lunar gravitational perturbation
geometry_msgs/Vector3 third_body_sun     # Solar gravitational perturbation
geometry_msgs/Vector3 solid_tides        # Solid Earth tides
geometry_msgs/Vector3 ocean_tides        # Ocean tides
geometry_msgs/Vector3 relativistic       # General relativistic effects
geometry_msgs/Vector3 thrust             # Active thrust/maneuver

# Perturbation magnitudes [m/s^2]
float64 mag_total
float64 mag_gravity
float64 mag_drag
float64 mag_srp
float64 mag_third_body
float64 mag_tides
