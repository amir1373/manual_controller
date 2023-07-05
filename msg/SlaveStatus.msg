std_msgs/Time stamp
bool stopped # if quick stop on = true otherwise false 
bool drives_powered # if switched on = true otherwise false
bool motion_possible # if operation enable = true otherwise false
bool in_error # if servo is fualt = true otherwise false
uint16 error_code # code of delta error in decimal 
string error_message  # message of delta error (translate decimal error code to humanreadable)