# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
uint16[] ranges         # LSB = 0.001m
int16[] angles          # LSB = 360/65536 degree
                        # def rad_to_int16(x):
                        #     value = int(x * 10430.378350470453)  # 65536.0 / (math.pi * 2)
                        #     return value % 65536
                        # def int16_to_rad(x):
                        #     return x * 9.587379924285257e-05  # (math.pi * 2 / 65536.0)
uint16[] time_deltas    # LSB = 0.0001s. Time delta from header.stamp
uint8[] intensities     # 0-255

float32 speed           # rad/s. Current rotation speed, reported by hardware, used for error reporting.