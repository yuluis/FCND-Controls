The minimum requirements for a successful submission include completing the following:

 - `body_rate_control()` - done
 - `altitude_control()` - done with approximation
 - `yaw_control()` - done
 - `roll_pitch_control()` - a reduced attitude controller taking in local acceleration or attitude commands and outputs body rate command.  Note that you will need to account for the non-linear transformation from local accelerations to body rates!
 - `lateral_position_control()` - a linear position controller using the local north/east position and local north/east velocity to generate a commanded local acceleration
 - The final moment/thrust commands limit the input at given saturation limits