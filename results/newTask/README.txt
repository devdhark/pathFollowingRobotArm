For this simulation, the specifications are,
T_se_initial = [[0, -1, 0, -0.5]; [0, 0, 1, 0.5]; [-1, 0, 0, 0.5]; [0, 0, 0, 1]];
T_sc_initial = [[0, -1, 0, -0.5]; [1, 0, 0, 1]; [0, 0, 1, 0.025]; [0 0 0 1]];
T_sc_final = [[1, 0, 0, 0.5]; [0, 1, 0, 0]; [0, 0, 1, 0.025]; [0 0 0 1]];

Initial cube configuration:- (-0.5 m, 1 m, +pi/2 rads)
Final cube configuration:- (0.5 m, 0 m, 0 rads)

Type of controller:- feedforward-plus-P
Kp = diag([1.5, 1.5, 1.5, 1.5, 1.5, 1.5]);
Ki = zeros(6);

new_config = [pi/4; -0.5; 0; -pi/6; -pi/4; -pi/4; -pi/4; 0; 0; 0; 0; 0];
