function dqBdes = dqBdesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    qBdes = atan2(zdes,xdes);

    xdes1 = traj(idx+1,1);
    zdes1 = traj(idx+1,2);
    qBdes1 = atan2(zdes1,xdes1);

    dt = 1/freq/length(traj(:,1));
    dqBdes = (qBdes1-qBdes)/dt;
end
