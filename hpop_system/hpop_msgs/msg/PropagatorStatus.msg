# Status of the orbit propagator system

std_msgs/Header header

# Propagator state
uint8 STATE_STOPPED = 0
uint8 STATE_RUNNING = 1
uint8 STATE_PAUSED = 2
uint8 STATE_ERROR = 3
uint8 state

# Timing
builtin_interfaces/Time simulation_time   # Current simulation epoch
builtin_interfaces/Time wall_clock_start  # When propagation started
float64 simulation_elapsed   # [s] Elapsed simulation time
float64 wall_clock_elapsed   # [s] Elapsed wall clock time
float64 time_ratio           # Ratio of sim time to wall clock (real-time factor)

# Satellites being tracked
uint32 num_satellites        # Number of active satellites
string[] satellite_ids       # List of satellite IDs

# Performance metrics
float64 update_rate_hz       # [Hz] Actual update rate
float64 avg_step_time_ms     # [ms] Average integration step time
float64 max_step_time_ms     # [ms] Maximum integration step time
uint64 total_steps           # Total integration steps performed
uint32 steps_per_second      # Integration steps per second

# Integrator configuration
string integrator_type       # RK4, RKF45, RKF78, ABM, etc.
float64 step_size            # [s] Integration step size
float64 tolerance            # Adaptive integrator tolerance

# Perturbation models active
bool gravity_enabled
uint32 gravity_degree        # Spherical harmonic degree (nmax)
bool drag_enabled
string atmosphere_model      # Exponential, NRLMSISE00, etc.
bool srp_enabled
bool third_body_moon_enabled
bool third_body_sun_enabled
bool solid_tides_enabled
bool ocean_tides_enabled

# Resource usage
float64 cpu_usage_percent    # CPU utilization
float64 memory_usage_mb      # Memory usage in MB
uint32 thread_count          # Number of active threads

# Errors and warnings
string last_error            # Last error message
string[] warnings            # Current warnings
uint32 error_count           # Total errors since start
