function ldes = ldesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    ldes = sqrt(xdes^2+zdes^2);
end



