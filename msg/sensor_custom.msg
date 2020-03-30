uint64 timestamp	# time since system start (microseconds)
uint32 device_id_acc	# unique device ID for the sensor that does not change between power cycles
uint32 device_id_gyr

uint64 error_count

float32 acc_x		# the raw acceleration (no cutoff filter) in the X board axis in m/s^2
float32 acc_y		# the raw acceleration (no cutoff filter) in the Y board axis in m/s^2
float32 acc_z		# the raw acceleration (no cutoff filter) in the Z board axis in m/s^2

float32 gry_x	# delta velocity in the X board axis in m/s over the integration time frame 
float32 gry_y	# delta velocity in the Y board axis in m/s over the integration time frame
float32 gry_z	# delta velocity in the Z board axis in m/s over the integration time frame

float32 temperature	# temperature in degrees celsius
#float32 scaling		# scaling from raw to m/s^s
#int16 x_raw
#int16 y_raw
#int16 z_raw
