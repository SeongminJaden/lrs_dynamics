# Two-Line Element Set data

std_msgs/Header header

# Identification
string satellite_name        # Satellite name (line 0)
uint32 norad_id              # NORAD Catalog Number
string intl_designator       # International Designator (launch year, number, piece)
string classification        # Classification (U=unclassified, C=classified, S=secret)

# Epoch
builtin_interfaces/Time epoch_time  # TLE epoch as ROS time
float64 epoch_year           # Year (2-digit in TLE, converted to 4-digit)
float64 epoch_day            # Day of year with fractional portion

# Mean motion derivatives
float64 mean_motion_dot      # [rev/day^2] First derivative of mean motion / 2
float64 mean_motion_ddot     # [rev/day^3] Second derivative of mean motion / 6
float64 bstar                # B* drag term [1/Earth radii]

# Orbital elements (at epoch)
float64 inclination          # [deg] Inclination
float64 raan                 # [deg] Right Ascension of Ascending Node
float64 eccentricity         # [-] Eccentricity (decimal point assumed)
float64 arg_periapsis        # [deg] Argument of Perigee
float64 mean_anomaly         # [deg] Mean Anomaly
float64 mean_motion          # [rev/day] Mean Motion

# Metadata
uint32 revolution_number     # Revolution number at epoch
uint32 element_set_number    # Element set number
float64 tle_age_days         # Age of TLE in days from current time

# Raw TLE lines
string line1
string line2
