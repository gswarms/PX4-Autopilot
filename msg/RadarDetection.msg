# Radar detection report
uint64 timestamp		# time since system start (microseconds)

uint8 track_id # Unique track ID - 1 for target, 2 for interceptor

float32[3] position         # Position in meters. NED earth-fixed frame. NaN if invalid/unknown
float32[3] velocity         # Velocity in meters/sec. NED earth-fixed frame. NaN if invalid/unknown

float32[3] position_variance
float32[3] velocity_variance
