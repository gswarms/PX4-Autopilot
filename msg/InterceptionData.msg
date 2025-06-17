# Radar detection report
uint64 timestamp		# time since system start (microseconds)

uint8 id # Unique ID for the interceptor

float32 tgo # time to go in seconds
float32 miss_distance # miss distance in meters
float32 position_std_norm # position standard deviation in meters

uint32 substate # interceptor substate

bool target_detected # true if the target is detected

float32[3] estimated_relative_position  # estimated relative position of the target in the body frame of the interceptor [m]