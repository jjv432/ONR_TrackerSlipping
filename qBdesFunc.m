function qBdes = qBdesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    qBdes = atan2(zdes,xdes) + pi;
end