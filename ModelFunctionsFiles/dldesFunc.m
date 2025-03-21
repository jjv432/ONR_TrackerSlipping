function dldes = dldesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    ldes = sqrt(xdes^2+zdes^2);

    xdes1 = traj(idx+1,1);
    zdes1 = traj(idx+1,2);
    ldes1 = sqrt(xdes1^2+zdes1^2);

    dt = 1/freq/length(traj(:,1));

    dldes = (ldes1-ldes)/dt;

end