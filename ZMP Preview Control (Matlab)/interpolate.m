function coordinate = interpolate(pos1,pos2,dt)

z_init = 0;
z_final = 0;
z_midway = 0.1; 

t_init = 0;
t_final = 0.8;
t_midway = (t_init + t_final)/2;

waypoints_z = [z_init z_midway z_final];
timestamp_z = [t_init t_midway t_final];

t_vector = timestamp_z(1):dt:timestamp_z(end);
[z, dz, ddz, ~] = cubicpolytraj(waypoints_z, timestamp_z, t_vector);

x_init = pos1(1);
x_final = pos2(1);
timestamp_x = [0 0.8];

waypoints_x = [x_init x_final];
[x, dx, ddx, ~] = cubicpolytraj(waypoints_x, timestamp_x, t_vector);

y_init = pos1(2);
y_final = pos2(2);
timestamp_y = [0 0.8];

waypoints_y = [y_init y_final];
[y, dy, ddy, ~] = cubicpolytraj(waypoints_y, timestamp_y, t_vector);

coordinate = [x(:),y(:),z(:)];

end
%plot(t_vector,z)