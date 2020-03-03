function new_config = NextState(new_config, speeds, dt, joint_lim, wheel_lim)
% Example:
% clear; clc;
% new_config= [0.5; 0; 0.5; 0; -pi/2; 0; 0; 0; 0; 0; 0; 0];
% speeds= [10; 10; 10; 10; 0.1; 1; 0.1; 0.1; 0.1];
% dt= 0.01;
% joint_lim= pi;
% wheel_lim= 5;
% for i=1:100
%     new_config = NextState(new_config, speeds, dt, joint_lim, wheel_lim);
%     mat(i,1:13)= [new_config; 0];
% end
% csvwrite('NextState.csv',mat);

phi = new_config(1);
x = new_config(2);
y = new_config(3);
j = speeds(5:9);
for i= 1:5      % Applying joint speed limits to the input joint speeds
    if(j(i) < 0)
        j(i) = max(j(i), -abs(joint_lim));
    else
        j(i) = min(j(i), abs(joint_lim));
    end
end
u = speeds(1:4);
for i= 1:4      % Applying wheel speed limits to the input wheel speeds
    if(u(i) < 0)
        u(i) = max(u(i), -abs(wheel_lim));
    else
        u(i) = min(u(i), abs(wheel_lim));
    end
end
% Formulation for odometry of the youBot chassis
l = 0.47/2;
w = 0.3/2;
r = 0.0475;
dtheta = u*dt;
Vb = (r/4)*[[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)];[1, 1, 1, 1];[-1, 1, -1, 1]]*dtheta;
if(NearZero(Vb(1)) == 1)
    dqb = [0; Vb(2); Vb(3)];
else
    dqb = [Vb(1); ...
         (Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1); ...
         (Vb(3)*sin(Vb(1))+Vb(2)*(1-cos(Vb(1))))/Vb(1)];
end
dq = [[1, 0, 0]; [0, cos(phi), -sin(phi)]; [0, sin(phi), cos(phi)]]*dqb;
new_chassis = new_config(1:3)+dq;       % Updated chassis configuration
new_joint = new_config(4:8)+dt*j;       % Updated joint configuration
new_wheel = new_config(9:12)+dt*u;       % Updated wheel configuration
new_config = [new_chassis; new_joint; new_wheel];
end