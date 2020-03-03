function traj_gen = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k)
% Example:
% clear; clc;
% T_se_initial = [[1 ,0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.5]; [0, 0, 0, 1]];
% T_sc_initial = [[1 ,0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 0.025]; [0, 0, 0, 1]];
% T_sc_final = [[0 ,1, 0, 0]; [-1, 0, 0, -1]; [0, 0, 1, 0.025]; [0, 0, 0, 1]];
% T_ce_grasp = [[-cos(pi/4), 0, sin(pi/4), 0]; [0, 1, 0, 0]; [-sin(pi/4), 0, -cos(pi/4), 0]; [0, 0, 0, 1]];
% T_ce_standoff = [[-cos(pi/4), 0, sin(pi/4), 0]; [0, 1, 0, 0]; [-sin(pi/4), 0, -cos(pi/4), 0.2]; [0, 0, 0, 1]];
% k=1;
% traj_gen = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k);

time = 18;  % Time taken(secs) to complete task excluding the gripper opening/closing time.
Tf = time/6;    % Time taken(secs) for each segment of the trajectory (Excluding gripper opening/closing).
gripper_time = 1;   % Time taken(secs) for the gripper to open/close.
Total_time = time+2*gripper_time;   % Total Time(secs) taken to perform the given task.
N = Tf*k/0.01;  % Number of  reference configurations for each segment of the trajectory.
N_grip = gripper_time*k/0.01;   % Number of reference configurations for the gripper open/close segments.
method = 3;
% Trajectory of end-effector to standoff position
traj = ScrewTrajectory(T_se_initial, T_sc_initial*T_ce_standoff, Tf, N, method);    % Using subscipt cancellation rule for transformation matrices
traj_gen=[];
for i=1:N
    r11=traj{i}(1,1); r12=traj{i}(1,2); r13=traj{i}(1,3); px=traj{i}(1,4);
    r21=traj{i}(2,1); r22=traj{i}(2,2); r23=traj{i}(2,3); py=traj{i}(2,4);
    r31=traj{i}(3,1); r32=traj{i}(3,2); r33=traj{i}(3,3); pz=traj{i}(3,4);
    traj_gen(i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,0];
end
% Trajectory of end-effector to grasping position
traj = ScrewTrajectory(T_sc_initial*T_ce_standoff, T_sc_initial*T_ce_grasp, Tf, N, method);
for i=1:N
    r11=traj{i}(1,1); r12=traj{i}(1,2); r13=traj{i}(1,3); px=traj{i}(1,4);
    r21=traj{i}(2,1); r22=traj{i}(2,2); r23=traj{i}(2,3); py=traj{i}(2,4);
    r31=traj{i}(3,1); r32=traj{i}(3,2); r33=traj{i}(3,3); pz=traj{i}(3,4);
    traj_gen(N+i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,0];
end
% Trajectory of end-effector clipper to close
for i=1:N_grip
traj_gen(2*N+i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,1];
end
% Trajectory of end-effector to return to standoff position
traj = ScrewTrajectory(T_sc_initial*T_ce_grasp, T_sc_initial*T_ce_standoff, Tf, N, method);
for i=1:N
    r11=traj{i}(1,1); r12=traj{i}(1,2); r13=traj{i}(1,3); px=traj{i}(1,4);
    r21=traj{i}(2,1); r22=traj{i}(2,2); r23=traj{i}(2,3); py=traj{i}(2,4);
    r31=traj{i}(3,1); r32=traj{i}(3,2); r33=traj{i}(3,3); pz=traj{i}(3,4);
    traj_gen(2*N+N_grip+i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,1];
end
% Trajectory of end-effector to standoff position of the final cube position
traj = ScrewTrajectory(T_sc_initial*T_ce_standoff, T_sc_final*T_ce_standoff, Tf, N, method);
for i=1:N
    r11=traj{i}(1,1); r12=traj{i}(1,2); r13=traj{i}(1,3); px=traj{i}(1,4);
    r21=traj{i}(2,1); r22=traj{i}(2,2); r23=traj{i}(2,3); py=traj{i}(2,4);
    r31=traj{i}(3,1); r32=traj{i}(3,2); r33=traj{i}(3,3); pz=traj{i}(3,4);
    traj_gen(3*N+N_grip+i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,1];
end
% Trajectory of end-effector to final cube position
traj = ScrewTrajectory(T_sc_final*T_ce_standoff, T_sc_final*T_ce_grasp, Tf, N, method);
for i=1:N
    r11=traj{i}(1,1); r12=traj{i}(1,2); r13=traj{i}(1,3); px=traj{i}(1,4);
    r21=traj{i}(2,1); r22=traj{i}(2,2); r23=traj{i}(2,3); py=traj{i}(2,4);
    r31=traj{i}(3,1); r32=traj{i}(3,2); r33=traj{i}(3,3); pz=traj{i}(3,4);
    traj_gen(4*N+N_grip+i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,1];
end
% Trajectory of end-effector clipper to open
for i=1:N
traj_gen(5*N+N_grip+i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,0];
end
% Trajectory of end-effector to return to standoff position of the final cube position
traj = ScrewTrajectory(T_sc_final*T_ce_grasp, T_sc_final*T_ce_standoff, Tf, N, method);
for i=1:N
    r11=traj{i}(1,1); r12=traj{i}(1,2); r13=traj{i}(1,3); px=traj{i}(1,4);
    r21=traj{i}(2,1); r22=traj{i}(2,2); r23=traj{i}(2,3); py=traj{i}(2,4);
    r31=traj{i}(3,1); r32=traj{i}(3,2); r33=traj{i}(3,3); pz=traj{i}(3,4);
    traj_gen(5*N+2*N_grip+i,1:13)=[r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,0];
end
csvwrite('TrajectoryGenerator.csv',traj_gen);    % Writes concatenated traj_gen of reference configurations into a csv file 
end