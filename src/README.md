
# README

Please note! When pulling this code, two changes need to be made to the WPI 32U4 Library.

1. First, in Chassis.h, the variables cmPerEncoderTick and robotRadius must be made public. 
2. Second, in Romi32U4Motors.h, the function void getCount() must also be made public.
