# Orbital maneuver plan

std_msgs/Header header

# Identification
string maneuver_id           # Unique maneuver identifier
string satellite_id          # Target satellite
string maneuver_name         # Human-readable name

# Maneuver type
uint8 TYPE_IMPULSIVE = 0     # Instantaneous delta-V
uint8 TYPE_FINITE_BURN = 1   # Continuous thrust
uint8 TYPE_LOW_THRUST = 2    # Electric propulsion
uint8 maneuver_type

# Timing
builtin_interfaces/Time ignition_time   # Maneuver start time
float64 burn_duration        # [s] Duration of burn (0 for impulsive)
builtin_interfaces/Time cutoff_time     # Maneuver end time

# Delta-V specification
geometry_msgs/Vector3 delta_v_eci       # [m/s] Delta-V in ECI frame
geometry_msgs/Vector3 delta_v_rtn       # [m/s] Delta-V in RTN frame
geometry_msgs/Vector3 delta_v_lvlh      # [m/s] Delta-V in LVLH frame
float64 delta_v_magnitude    # [m/s] Total delta-V magnitude

# RTN components (convenience)
float64 dv_radial            # [m/s] Radial component
float64 dv_intrack           # [m/s] In-track (tangential) component
float64 dv_crosstrack        # [m/s] Cross-track (normal) component

# Attitude during maneuver
geometry_msgs/Quaternion thrust_attitude  # Thrust direction attitude
float64 thrust_azimuth       # [deg] Thrust azimuth in body frame
float64 thrust_elevation     # [deg] Thrust elevation in body frame

# Propulsion parameters
float64 thrust_magnitude     # [N] Thrust force
float64 specific_impulse     # [s] Engine Isp
float64 propellant_mass      # [kg] Required propellant

# Pre/post maneuver states
hpop_msgs/OrbitalElements pre_maneuver_elements
hpop_msgs/OrbitalElements post_maneuver_elements

# Maneuver purpose
uint8 PURPOSE_ORBIT_RAISE = 0
uint8 PURPOSE_ORBIT_LOWER = 1
uint8 PURPOSE_PLANE_CHANGE = 2
uint8 PURPOSE_RENDEZVOUS = 3
uint8 PURPOSE_COLLISION_AVOID = 4
uint8 PURPOSE_STATION_KEEP = 5
uint8 PURPOSE_DEORBIT = 6
uint8 PURPOSE_PHASING = 7
uint8 purpose

# Status
uint8 STATUS_PLANNED = 0
uint8 STATUS_APPROVED = 1
uint8 STATUS_UPLOADED = 2
uint8 STATUS_EXECUTING = 3
uint8 STATUS_COMPLETED = 4
uint8 STATUS_ABORTED = 5
uint8 status

# Constraints and margins
float64 delta_v_margin       # [m/s] Additional margin included
float64 pointing_accuracy    # [deg] Required pointing accuracy
string constraints_notes     # Notes about constraints
