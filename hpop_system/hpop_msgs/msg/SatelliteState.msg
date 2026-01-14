# Complete satellite state including position, velocity, attitude, and properties

std_msgs/Header header

# Identification
string satellite_id          # Unique identifier
uint32 norad_id              # NORAD catalog number (0 if not assigned)
string name                  # Human-readable name

# Reference frame
string frame_id              # ECI_J2000, ECEF_ITRF, etc.

# Translational state
geometry_msgs/Point position          # [m] Position vector
geometry_msgs/Vector3 velocity        # [m/s] Velocity vector

# Rotational state (for 6DOF simulation)
geometry_msgs/Quaternion orientation  # Body to reference frame rotation
geometry_msgs/Vector3 angular_velocity # [rad/s] Angular velocity in body frame

# Orbital elements (derived from state)
hpop_msgs/OrbitalElements elements

# Physical properties
float64 mass                  # [kg] Spacecraft mass
float64 cross_section_drag    # [m^2] Cross-sectional area for drag
float64 cross_section_srp     # [m^2] Cross-sectional area for SRP
float64 drag_coefficient      # [-] Drag coefficient (Cd), typically 2.0-2.5
float64 reflectivity_coeff    # [-] Reflectivity coefficient (Cr), typically 1.0-2.0

# Propagation metadata
builtin_interfaces/Time propagation_epoch  # Reference epoch for propagation
float64 propagation_elapsed   # [s] Time since propagation start
uint32 integration_steps      # Number of integration steps taken

# Status flags
bool is_valid                 # State validity flag
bool in_shadow                # True if satellite is in Earth's shadow
bool in_contact               # True if in contact with any ground station
