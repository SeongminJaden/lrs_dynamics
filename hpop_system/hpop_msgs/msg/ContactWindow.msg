# Contact window between a satellite and ground station

std_msgs/Header header

# Participants
string satellite_id          # Satellite identifier
string station_id            # Ground station identifier

# Time window
builtin_interfaces/Time aos  # Acquisition of Signal (start of contact)
builtin_interfaces/Time los  # Loss of Signal (end of contact)
builtin_interfaces/Time tca  # Time of Closest Approach (max elevation)

# Duration
float64 duration_seconds     # [s] Total contact duration

# Geometry at TCA
float64 max_elevation        # [deg] Maximum elevation angle
float64 azimuth_aos          # [deg] Azimuth at AOS
float64 azimuth_los          # [deg] Azimuth at LOS
float64 range_min            # [m] Minimum range to satellite

# Quality metrics
float64 contact_quality      # [0-1] Overall contact quality score
bool is_viable               # True if contact meets minimum requirements
string quality_notes         # Notes about contact quality

# Scheduling
uint32 contact_number        # Contact sequence number
uint32 orbit_number          # Satellite orbit number during contact
bool is_scheduled            # True if contact is scheduled for operations
