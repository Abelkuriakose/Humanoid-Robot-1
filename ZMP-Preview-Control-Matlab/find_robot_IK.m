function solution = find_robot_IK(l_fs_com,r_fs_com,robot)

l_tform = trvec2tform(l_fs_com);
r_tform = trvec2tform(r_fs_com);

ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1];
guess = robot.homeConfiguration;
solution = [];

for i=1:size(l_fs_com)
    [l_Soln,l_solnInfo] = ik('l_leg_foot_link',l_tform(:,:,i),weights,guess);
    
    [r_Soln,r_solnInfo] = ik('r_leg_foot_link',r_tform(:,:,i),weights,guess);

    guess = [l_Soln(1,1:6) r_Soln(1,7:end)];
    solution = [solution; guess];
    
%joint_solution = [0];
end

end