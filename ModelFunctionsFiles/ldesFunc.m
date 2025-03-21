function ldes = ldesFunc(t,obj)
    idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
    if(idx==0)
        idx = 1;
    end
    xdes = obj.ODEVariables.traj(idx,1);
    zdes = obj.ODEVariables.traj(idx,2);
    ldes = sqrt(xdes^2+zdes^2);
end



