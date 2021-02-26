function [com_x, com_y, p_x, p_y, sst] = calc_preview_control(zmp_x, zmp_y, dt, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gd)

% State for com x and y direction
x_x = [0;0;0];
x_y = [0;0;0];

% Trajectory of left and right foot
sst = [];
time = 0;
ssp = 1;
dsp = 2;
sp = dsp;

% Variable for plotting
com_x = [];
com_y = [];
p_x = [];
p_y = [];

i = 1;
for ii=0:dt:t_calc
    % y = Cx
    % y = output zmp
    y_x = C_d * x_x;
    y_y = C_d * x_y;
    
    % calculate error 
    % target zmp - current zmp
    e_x = zmp_x(i) - y_x;
    e_y = zmp_y(i) - y_y;
    
    % preview horizon
    preview_x = 0;
    preview_y = 0;
    j = 1;

    for n=ii:dt:(ii+t_preview)
        preview_x = preview_x + Gd(j) * zmp_x(i+j);
        preview_y = preview_y + Gd(j) * zmp_y(i+j);
        j = j+1;
    end
    
    % pick signal input u
    u_x = -Gi * e_x - Gx * x_x - preview_x;
    u_y = -Gi * e_y - Gx * x_y - preview_y;
    
    % update state
    x_x = A_d * x_x + B_d * u_x;
    x_y = A_d * x_y + B_d * u_y;
    
    % update output    
    y_x = C_d * x_x;
    y_y = C_d * x_y;
    
    p_x = [p_x y_x];
    p_y = [p_y y_y];
    
    % save current state to array
    com_x = [com_x x_x(1)];
    com_y = [com_y x_y(1)];
    
    %& (ii < (2.0/dt)))
    if (abs(y_y) > 0.0435 & sp == dsp)
        sst = [sst time];
        sp = ssp;
    end
    if (abs(y_y) < 0.0435)
        sp = dsp;
    end
    
    time = time + dt;
    i = i + 1;
end
%display(sst)
end

