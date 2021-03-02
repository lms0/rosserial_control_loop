# rosserial_control_loop

Source code for rosserial node. Subscribed to the cmd_vel topic, from which it receives a velocity reference <linear, angular> of type geometry_msgs/Twist.

It converts this velocity into differential velocities for the two robot actuators (Vr right, Vl left).

The reference is passed through a PID algorithm (negative feedback), and the motors are driven to follow the reference.
