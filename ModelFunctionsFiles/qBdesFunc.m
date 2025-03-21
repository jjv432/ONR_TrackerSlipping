function qBdes = qBdesFunc(t,obj)
    idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
    if(idx==0)
        idx = 1;
    end
    xdes = obj.ODEVariables.trajtraj(idx,1);
    zdes = obj.ODEVariables.trajtraj(idx,2);
    qBdes = atan2(zdes,xdes) + pi;
end