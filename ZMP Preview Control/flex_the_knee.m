function [l_fs_comf, r_fs_comf] = flex_the_knee(com_x,com_y,lfs,rfs,zc,z0,kn_time,dt)

%For CoM
waypoints_z = [zc z0];
timestamp = [0 kn_time];                   % Time for bending
t_vector = timestamp(1):dt:timestamp(end);

[z, dz, ddz, ~] = cubicpolytraj(waypoints_z, timestamp, t_vector);

waypoints_x = [com_x com_x];
waypoints_y = [com_y com_y];
[x, dx, ddx, ~] = cubicpolytraj(waypoints_x, timestamp, t_vector);
[y, dy, ddy, ~] = cubicpolytraj(waypoints_y, timestamp, t_vector);

com = [x' y' z'];
%For Left Legs
waypoints_z = [0 0];
timestamp = [0 kn_time];                   % Time for bending
t_vector = timestamp(1):dt:timestamp(end);

[z, dz, ddz, ~] = cubicpolytraj(waypoints_z, timestamp, t_vector);

waypoints_x = [lfs(1) lfs(1)];
waypoints_y = [lfs(2) lfs(2)];
[x, dx, ddx, ~] = cubicpolytraj(waypoints_x, timestamp, t_vector);
[y, dy, ddy, ~] = cubicpolytraj(waypoints_y, timestamp, t_vector);

l_fs = [x' y' z'];

% For right Leg
waypoints_z = [0 0];
timestamp = [0 kn_time];                   % Time for bending
t_vector = timestamp(1):dt:timestamp(end);

[z, dz, ddz, ~] = cubicpolytraj(waypoints_z, timestamp, t_vector);

waypoints_x = [rfs(1) rfs(1)];
waypoints_y = [rfs(2) rfs(2)];
[x, dx, ddx, ~] = cubicpolytraj(waypoints_x, timestamp, t_vector);
[y, dy, ddy, ~] = cubicpolytraj(waypoints_y, timestamp, t_vector);

r_fs = [x' y' z'];

l_fs_comf = l_fs - com;
r_fs_comf = r_fs - com;

end