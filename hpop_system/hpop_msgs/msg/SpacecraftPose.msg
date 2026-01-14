# 6DOF spacecraft pose for MoveIt2 integration and rendezvous operations

std_msgs/Header header

# Frame definitions
string frame_id              # Parent frame: ECI, ECEF, LVLH, etc.
string child_frame_id        # This spacecraft's body frame

# Pose (position + orientation)
geometry_msgs/Pose pose

# Twist (linear + angular velocity)
geometry_msgs/Twist twist

# Covariance matrices for navigation filter (row-major)
# Order: x, y, z, roll, pitch, yaw
float64[36] pose_covariance
float64[36] twist_covariance

# For relative navigation (rendezvous operations)
string reference_spacecraft_id  # ID of reference spacecraft (empty if absolute)
bool is_relative              # True if pose is relative to reference spacecraft

# LVLH frame state (when frame_id is LVLH)
# R: radial (positive outward from Earth)
# T: along-track (positive in velocity direction)
# N: cross-track (completes right-hand system)
geometry_msgs/Vector3 rtn_position   # [m] Position in RTN frame
geometry_msgs/Vector3 rtn_velocity   # [m/s] Velocity in RTN frame
