# Proximity event between two satellites (conjunction/rendezvous)

std_msgs/Header header

# Object identification
string primary_id            # Primary satellite ID (typically the protected asset)
string secondary_id          # Secondary satellite/debris ID
uint32 primary_norad         # Primary NORAD ID
uint32 secondary_norad       # Secondary NORAD ID

# Time of Closest Approach (TCA)
builtin_interfaces/Time tca
float64 time_to_tca          # [s] Time until TCA (negative if past)

# Miss distance
float64 miss_distance        # [m] Minimum distance at TCA
float64 radial_miss          # [m] Radial component (R in RTN)
float64 intrack_miss         # [m] In-track component (T in RTN)
float64 crosstrack_miss      # [m] Cross-track component (N in RTN)

# Relative state in LVLH/RTN frame at TCA
geometry_msgs/Point relative_position   # [m] Relative position
geometry_msgs/Vector3 relative_velocity # [m/s] Relative velocity
float64 approach_speed       # [m/s] Closing speed

# Collision assessment
float64 collision_probability  # [0-1] Probability of collision (Pc)
float64 combined_hard_body_radius  # [m] Sum of object radii
bool is_collision_risk       # True if Pc > threshold

# Covariance information (for Pc calculation)
float64[9] position_covariance_primary    # 3x3 covariance matrix
float64[9] position_covariance_secondary  # 3x3 covariance matrix
float64[9] combined_covariance            # Combined covariance at TCA

# Event classification
uint8 EVENT_TYPE_CONJUNCTION = 0
uint8 EVENT_TYPE_RENDEZVOUS = 1
uint8 EVENT_TYPE_PROXIMITY_OPS = 2
uint8 event_type

# Action recommendations
bool maneuver_recommended    # True if avoidance maneuver recommended
string recommended_action    # Description of recommended action
