# Radar detection report
uint64 timestamp		# time since system start (microseconds)

uint8 id # Unique ID for the interceptor

float32 tgo # time to go in seconds
float32 miss_distance # miss distance in meters
float32 target_time_delay # time delay of the target measurements in seconds
float32 position_std_norm # position standard deviation in meters

uint32 substate # interceptor substate

bool target_detected # true if the target is detected

float32[3] estimated_relative_position  # estimated relative position of the target in the body frame of the interceptor [m]

bool interceptor_active # flag if the interceptor node is ACTIVE
bool target_estimation_active # flag if the target_estimation node is ACTIVE
bool camera_driver_active # flag if the camera_driver node is ACTIVE
bool monitor_active # flag if the monitor node is ACTIVE
bool recorder_active # flag if the recorder node is ACTIVE
bool detector_active # flag if the detector node is ACTIVE
bool osd_active # flag if the osd node is ACTIVE