clc;
clear;
disp("Test Preview Control");
animate = 3;

%import the robot urdf model
robot = importrobot("urdf\generated.urdf");

z0 = 0.78; % standing height
zc = 0.611131; % LIPM height
dt = 0.01; % delta time (s)
t_step = 0.9; % timing for one step (s)
t_preview = 1; % timing for preview (s)
t_calc = 2; % timing parameter for simulation (s), this value will be override automatically by calculation to prevent error
kn_time = 1.3;  % time for bending the knee

% Need to be tuned manually
% Tune these two parameters until you get proper CoM trajectory
Qe = 1;
R = 1e-2;

% Given footstep input
% x, y, and theta
% footstep = [0.00, 0.00, 0.00;
%             0.25, 0.10, 0.00;
%             0.50, -0.10, 0.00;
%             0.75, 0.10, 0.00;
%             1.00, -0.10, 0.00];
   
footstep = [0.0 0.0 0.0; 
            0.2 0.06 0.0; 
            0.4 -0.06 0.0; 
            0.6 0.09 0.0; 
            0.8 -0.03 0.0; 
            1.3 0.09 0.0; 
            1.7 -0.03 0.0; 
            1.9 0.09 0.0; 
            2.0 -0.03 0.0];
 y = 0.0915;
 x_step = 0.416;
 footstep = [ 0.0           0.0;        % pos1
              0.            y;          % pos2
              x_step/2      -y;         % pos3
              x_step        y;          % pos4
              1.5*x_step    -y;         % pos5
              2*x_step      y;          % pos6
              2.5*x_step    -y;         % pos7
              3*x_step      y;          % pos8
              3.5*x_step    -y;         % pos9
              3.5*x_step    0 ;         % pos10
              3.5*x_step    0 ;         % pos11
              3.5*x_step    0 ;         % pos12
              ];

lfootstep = footstep(2:2:end-3,:);
lfootstep = [lfootstep; 1.4560 y];
rfootstep = [0 -y];
rfootstep = [rfootstep; footstep(3:2:end-2,:)];

% Automatic calculation for simulation time
t_calc = length(footstep) * t_step - t_preview - dt;

% Generating ZMP trajectory
[zmp_x, zmp_y] = create_zmp_trajectory(footstep, dt, t_step);

% Getting parameter and gain
[A_d, B_d, C_d, Gi, Gx, Gd] = get_preview_control_parameter(zc, dt, t_preview, Qe, R);

% Simulating for t_calc seconds
[com_x, com_y, p_x, p_y, sst] = calc_preview_control(zmp_x, zmp_y, dt, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gd);
%display(sst);
rsst = sst(1:2:end);
lsst = sst(2:2:end);
% Generating Footstep trajectory
[l_fs, r_fs] = calc_fs_trajectory(lfootstep, rfootstep, dt, lsst, rsst,t_calc);
l_fs = l_fs(1:980,:);
r_fs = r_fs(1:980,:);

[l_fs_com, r_fs_com] = cal_fs_com(l_fs,r_fs,com_x,com_y,zc);
[l_fs_com0,r_fs_com0] = bend_the_knee(lfootstep(1,:),rfootstep(1,:),zc,z0,kn_time,dt);
[l_fs_comf, r_fs_comf] = flex_the_knee(com_x(end),com_y(end),lfootstep(end,:),rfootstep(end,:),zc,z0,kn_time,dt);
l_fs_com = [l_fs_com0; l_fs_com ; l_fs_comf];
r_fs_com = [r_fs_com0; r_fs_com; r_fs_comf];

%Inverse Kinematics
conf_solution = find_robot_IK(l_fs_com,r_fs_com,robot);

if animate == 0
    % Plot ZMP and CoM trajectory
    figure('name','ZMP X-Axis');
    hold;
    title('x components vs time')
    plot(zmp_x,'DisplayName','desired x ZMP');
    plot(com_x, '--','DisplayName','generated x CoM');
    plot(p_x, '-g','DisplayName','generated x ZMP');
    set(gca,'XTick',0:100:800)
    set(gca,'XTickLabel',0:1:8)
    xlabel('time(s)')
    ylabel('x components(m)')
    set(gca,'FontSize',13)
    legend

    figure('name','ZMP Y-Axis');
    hold;
    title('y components vs time')
    plot(zmp_y,'DisplayName','desired y ZMP');
    plot(com_y, '--','DisplayName','generated y CoM');
    plot(p_y, '-g', 'DisplayName','generated y ZMP');
    set(gca,'XTick',0:100:800)
    set(gca,'XTickLabel',0:1:8)
    xlabel('time(s)')
    ylabel('y components(m)')
    set(gca,'FontSize',13)
    legend

    figure('name','ZMP VS CoM');
    hold;
    title('Leg and CoM patterns generated')
    h = plot(zmp_x, zmp_y,'.','DisplayName','desired ZMP points');
    set(h,'MarkerSize',50)
    plot(com_x, com_y,'DisplayName','generated CoM xy trajectory');
    plot(p_x,p_y,'--k','DisplayName','generated ZMP xy trajectory')
    xlabel('x co-ordinate(m)')
    ylabel('y co-ordinate(m)')
    set(gca,'FontSize',13)
    legend

    figure('name','Foot Trajectory');
    hold;
    title('Foot Trajectory');
    plot3(l_fs(:,1), l_fs(:,2),l_fs(:,3),'DisplayName','left foot');
    plot3(r_fs(:,1), r_fs(:,2),r_fs(:,3),'--k','DisplayName','right foot');
    xlabel('x co-ordinate(m)');
    ylabel('y co-ordinate(m)');
    zlabel('z co-ordinate(m)');
    set(gca,'FontSize',13);
    legend;
    grid on;

elseif animate == 1
    figure('name','Foot Trajectory');
    hold;
    for k = 1:length(r_fs)
        plot3(l_fs(1:k,1), l_fs(1:k,2),l_fs(1:k,3),'DisplayName','left foot');
        hold;
        plot3(r_fs(1:k,1), r_fs(1:k,2),r_fs(1:k,3),'--k','DisplayName','right foot');
        hold;
        axis([0 1.5 -0.1 0.1 0 0.2]);
        set(gca,'FontSize',13);
        title('Foot Trajectory');
        xlabel('x co-ordinate(m)');
        ylabel('y co-ordinate(m)');
        zlabel('z co-ordinate(m)');
        legend;
        grid on;
        view(45,45);
        pause(0.001);
        if k ~=length(r_fs)
            clf;
        end
    end
elseif animate == 2
    % Testing Real Robot model
    display("Inverse Knematics");
    for i = 1:length(conf_solution(:,1))
        show(robot,conf_solution(i,:));
        pause(0.01)
    end

end


% Save parameter to mat file
% We can import this parameter to Python or C++ for real usage
save('conf_sol_4_thormang3.mat','conf_solution');
