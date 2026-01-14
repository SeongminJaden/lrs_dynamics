# Ground station definition

string station_id            # Unique identifier
string name                  # Human-readable name

# Geodetic coordinates
float64 latitude             # [deg] Geodetic latitude (-90 to 90)
float64 longitude            # [deg] Geodetic longitude (-180 to 180)
float64 altitude             # [m] Altitude above WGS84 ellipsoid

# ECEF coordinates (computed from geodetic)
geometry_msgs/Point ecef_position  # [m] Position in ECEF frame

# Antenna/tracking constraints
float64 min_elevation        # [deg] Minimum elevation angle for contact
float64 max_elevation        # [deg] Maximum elevation angle (typically 90)
float64 min_azimuth          # [deg] Minimum azimuth (0 if omnidirectional)
float64 max_azimuth          # [deg] Maximum azimuth (360 if omnidirectional)

# Communication capabilities
float64 frequency_min        # [Hz] Minimum supported frequency
float64 frequency_max        # [Hz] Maximum supported frequency
float64 antenna_gain         # [dBi] Antenna gain
float64 transmit_power       # [W] Transmit power

# Operational parameters
bool is_active               # Station is currently operational
string[] supported_satellites  # List of satellite IDs this station can track
