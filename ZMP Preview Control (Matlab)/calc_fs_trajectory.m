function [l_fs, r_fs] = calc_fs_trajectory(lfootstep, rfootstep, dt, lsst, rsst,t_calc)

time = 0;
l_fs = [];
r_fs = [];
index = 1;

terminate = 0;
%Right foot trajectory
while time < t_calc
    if(time > rsst(index))
        %display(r_fs);
        r_fs_temp = interpolate(rfootstep(index+terminate,:),rfootstep(index+1,:),dt);
        r_fs = [r_fs; r_fs_temp(:,1) r_fs_temp(:,2) (1-terminate)*r_fs_temp(:,3)];
        time = time + 0.8;
        if index ~= length(rsst)
            index = index + 1;
        else
            terminate = 1;
        end
    else
        r_fs = [r_fs; rfootstep(index,:) 0];
    end
    time = time + dt;
end

index = 1;
time = 0;
terminate = 0;
%Left foot trajectory
while time < t_calc
    if(time > lsst(index))
        l_fs_temp = interpolate(lfootstep(index+terminate,:),lfootstep(index+1,:),dt);
        l_fs = [l_fs; l_fs_temp(:,1) l_fs_temp(:,2) (1-terminate)*l_fs_temp(:,3)];
        time = time + 0.8;
        if index ~= length(lsst)
            index = index + 1;
        else
            terminate = 1;
        end
    else
        l_fs = [l_fs; lfootstep(index,:) 0];
    end
    time = time + dt;
end
end