For this simulation, the specifications are:

T_se_initial = [[0, 0, 1, 0]; [0, 1, 0, 0]; [-1, 0, 0, 0.5]; [0, 0, 0, 1]];

Initial Cube Configuration:- (1 m, 0 m, 0 rads)
Final Cube Configuration:- (0 m, -1 m, -pi/2 rads)

Type of controller:- feedforward-plus-P
Kp = diag([1.5,1.5,1.5,1.5,1.5,1.5]);
Ki = zeros(6);

new_config = [-pi/4; 0; 0.5; -pi/6; -pi/4; -pi/4; -pi/4; 0; 0; 0; 0; 0];
