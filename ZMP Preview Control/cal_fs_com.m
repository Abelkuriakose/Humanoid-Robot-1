function [l_fs_com, r_fs_com] = cal_fs_com(l_fs,r_fs,com_x,com_y,zc)

com_z = zc*ones(length(com_x),1);
com = [com_x', com_y', com_z];


l_fs_com = l_fs - com;
r_fs_com = r_fs - com;