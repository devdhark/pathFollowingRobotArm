clear;clc;
T_se_initial = [[0, -1, 0, -0.5]; [0, 0, 1, 0.5]; [-1, 0, 0, 0.5]; [0, 0, 0, 1]];
T_sc_initial = [[0, -1, 0, -0.5]; [1, 0, 0, 1]; [0, 0, 1, 0.025]; [0 0 0 1]];
T_sc_final = [[1, 0, 0, 0.5]; [0, 1, 0, 0]; [0, 0, 1, 0.025]; [0 0 0 1]];
T_ce_grasp = [[-cos(pi/4), 0, sin(pi/4), 0]; [0, 1, 0, 0]; [-sin(pi/4), 0, -cos(pi/4), 0]; [0, 0, 0, 1]];
T_ce_standoff = [[-cos(pi/4), 0, sin(pi/4), 0]; [0, 1, 0, 0]; [-sin(pi/4), 0, -cos(pi/4), 0.2]; [0, 0, 0, 1]];
k = 1;
% Trajectory of end-effector
traj_gen = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff,k);
[rows, columns] = size(traj_gen);
M = [[1, 0, 0, 0.033]; [0, 1, 0, 0]; [0, 0, 1, 0.6546]; [0, 0, 0, 1]];
Blist_arm = [[0, 0, 0, 0, 0]; ...
         [0, -1, -1, -1, 0]; ...
         [1, 0, 0, 0, 1]; ...
         [0, -0.5076, -0.3526, -0.2176, 0]; ...
         [0.033, 0, 0, 0, 0]; ...
         [0, 0, 0, 0, 0]];
dt = 0.01;
Kp = diag([1.5, 1.5, 1.5, 1.5, 1.5, 1.5]);
Ki = zeros(6);
% Initial youBot configuration 
new_config = [pi/4; -0.5; 0; -pi/6; -pi/4; -pi/4; -pi/4; 0; 0; 0; 0; 0];
joint_lim = 1000;
wheel_lim = 1000;
mat(1,1:13) = [new_config; 0];
for i = 1:rows-1
    Xd = [[traj_gen(i,1), traj_gen(i,2), traj_gen(i,3), traj_gen(i,10)];[traj_gen(i,4), traj_gen(i,5), traj_gen(i,6), traj_gen(i,11)]; ...
          [traj_gen(i,7), traj_gen(i,8), traj_gen(i,9), traj_gen(i,12)];[0, 0, 0, 1]];
    Xd_next = [[traj_gen(i+1,1), traj_gen(i+1,2), traj_gen(i+1,3), traj_gen(i+1,10)];[traj_gen(i+1,4), traj_gen(i+1,5), traj_gen(i+1,6), traj_gen(i+1,11)]; ...
               [traj_gen(i+1,7), traj_gen(i+1,8), traj_gen(i+1,9), traj_gen(i+1,12)];[0, 0, 0, 1]];
    % Formulation to find the actual end-effector configuration
    phi = new_config(1);
    x = new_config(2);
    y = new_config(3);
    thetalist_arm = new_config(4:8);
    Tsb = [[cos(phi), -sin(phi), 0, x]; [sin(phi), cos(phi), 0, y]; [0, 0, 1, 0.0963]; [0, 0, 0, 1]];
    Tb0 = [[1, 0, 0, 0.1662]; [0, 1, 0, 0]; [0, 0, 1, 0.0026]; [0, 0, 0, 1]];
    Toe = FKinBody(M, Blist_arm, thetalist_arm);
    X = Tsb*Tb0*Toe;
    [V, speeds, X_err] = FeedbackControl(Xd, Xd_next, X, Kp, Ki, dt, new_config);
    % Creates a matrix with all X_err produced by FeedbackControl
    Xerr_plot(i,1:6) = [X_err];
    new_config = NextState(new_config, speeds, dt, joint_lim, wheel_lim);
    % Creates a matrix to store the youBot configurations.
    mat(i+1,1:13) = [new_config; traj_gen(i,13)];    
end
csvwrite('project.csv',mat);
csvwrite('X_errPlot.csv',Xerr_plot);
% Plotting X_err w.r.t time
plot(Xerr_plot,'LineWidth',2)
title('X err Plot')
xlabel('time (tens of milliseconds)')
ylabel('X err')
legend('Element 1', 'Element 2', 'Element 3', 'Element 4', 'Element 5', 'Element 6')