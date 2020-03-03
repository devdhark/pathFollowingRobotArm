For this simulation, the specifications are:

T_se_initial = [[0, 0, 1, 0]; [0, 1, 0, 0]; [-1, 0, 0, 0.5]; [0, 0, 0, 1]];

Initial Cube Configuration:- (1 m, 0 m, 0 rads)
Final Cube Configuration:- (0 m, -1 m, -pi/2 rads)

Type of controller:- only-PI
Kp = diag([3, 3, 3, 3, 3, 3]);
Ki = eye(6);

To acheive a PI controller, in the function FeedbackControl the term Vd is taken to be zero(Uncomment line 19).
The X_err Plot shown in this case produces a non-zero error during the trajectory segment 5(Initial standoff position to final standoff position). I suspect this is because without the feedforward term the error twist is not being fully corrected for this trajectory. Although the predetermined trajectory is still being followed by the end-effector.

new_config = [-pi/4; 0; 0.5; -pi/6; -pi/4; -pi/4; -pi/4; 0; 0; 0; 0; 0];

