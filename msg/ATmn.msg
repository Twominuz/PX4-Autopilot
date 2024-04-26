uint64 timestamp	# time since system start (microseconds)
char[10] name           # max. 10 characters as key / name
float32 x               # x value
float32 y               # y value
float32 z               # z value

float32 remaining			# From 1 to 0, -1 if unknown
float32 scale				# Power scaling factor, >= 1, or -1 if unknown
float32 time_remaining_s	# predicted time in seconds remaining until battery is empty under previous averaged load, NAN if unknown
float32 temperature			# temperature of the battery. NaN if unknown
uint8 cell_count			# Number of cells, 0 if unknown
