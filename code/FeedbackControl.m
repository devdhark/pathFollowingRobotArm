function [V, speeds, X_err] = FeedbackControl(Xd, Xd_next, X, Kp, Ki, dt, new_config)
% Formulation for the end-effector twist V
phi = new_config(1);
x = new_config(2);
y = new_config(3);
M = [[1, 0, 0, 0.033]; [0, 1, 0, 0]; [0, 0, 1, 0.6546]; [0, 0, 0, 1]];
Blist_arm = [[0, 0, 0, 0, 0]; ...
         [0, -1, -1, -1, 0]; ...
         [1, 0, 0, 0, 1]; ...
         [0, -0.5076, -0.3526, -0.2176, 0]; ...
         [0.033, 0, 0, 0, 0]; ...
         [0, 0, 0, 0, 0]];
thetalist_arm = new_config(4:8);
Tsb = [[cos(phi), -sin(phi), 0, x]; [sin(phi), cos(phi), 0, y]; [0, 0, 1, 0.0963]; [0, 0, 0, 1]];
Tb0 = [[1, 0, 0, 0.1662]; [0, 1, 0, 0]; [0, 0, 1, 0.0026]; [0, 0, 0, 1]];
Toe = FKinBody(M, Blist_arm, thetalist_arm);
X_err = se3ToVec(MatrixLog6(inv(X)*Xd));
Vd = se3ToVec((1/dt)*(MatrixLog6(inv(Xd)*Xd_next)));
% Vd=zeros(6,1);    % For the case runovershoot
V = Adjoint(inv(X)*Xd)*Vd+ Kp*X_err+ Ki*X_err*dt;
% Formulation of finding the speeds required.
J_arm = JacobianBody(Blist_arm, thetalist_arm);
l= 0.47/2;
w= 0.3/2;
r= 0.0475;
F = (r/4)*[[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)];[1, 1, 1, 1];[-1, 1, -1, 1]];
F6 = [zeros(1,4); zeros(1,4); F; zeros(1,4)];
J_base = Adjoint(inv(Toe)*inv(Tb0))*F6;
Je = [J_base J_arm];
% Stops the robot arm to reach a singularity
if (abs(new_config(6))<pi/12)
    Je(:,6) = 0;
elseif (abs(new_config(7))<pi/12)
    Je(:,7) = 0;
end
speeds = pinv(Je, 1e-4)*V;
end