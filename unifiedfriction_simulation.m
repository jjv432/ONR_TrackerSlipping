clc; clear; close all; format compact

obj = getParams();


%%

global isSliding
isSliding = 0;

load("T_Results.mat");

t_results = t(15);
StatsLength = t_results.StatsPlottingTrialLength;
cur_t_vals = t_results.t(1:StatsLength);
cur_x_vals = t_results.MeanXPosition(1:StatsLength);
% cur_t_vals = t_results.t(1:t_results.StatsPlottingTrialLength);
% cur_x_vals = t_results.MeanXPosition(1:t_results.StatsPlottingTrialLength);

p = polyfit(cur_t_vals, cur_x_vals, 13);


% initial conditions
l0 = ldesFunc(0, obj);
qB0 = qBdesFunc(0, obj);

% Should vary with frequency

%  FIX THIS!!!!!
obj.ODEVariables.tspan = t_results.t(1:StatsLength/3);

%% Testing

% parm = [0 3.7762 80 10 1.6924];
%
% [x t] = UnifiedModelLight(parm, obj, 0);
% figure()
% plot(x,t)



%% Run PSO

costFunctionHandle = @(freeParams) ModelSimulationCost(freeParams, obj, p);

disp("Starting PSO...");
psoTimer = tic;
[OptimizedState, FVAL] = particleswarm(costFunctionHandle, obj.PSOInfo.nvars, obj.PSOInfo.LB, obj.PSOInfo.UB, obj.PSOInfo.options);
fprintf("\n\nPSO Completed in %.3f seconds", toc(psoTimer));

disp(OptimizedState);

[footPos, time] = UnifiedModelLight(OptimizedState, obj, 1);

figure
hold on
plot(time, footPos, 'xr');
plot(cur_t_vals, cur_x_vals, 'k');
legend("Best Simulation", "Experimental Data");
%%

function [Cost] = ModelSimulationCost(freeParams, obj, p)

    timerVal = tic;
    fprintf("Starting a Simulation...");

    % Run the simulation to determine the range of x values
    [simulation_x_array, simulation_time, warning] = UnifiedModelLight(freeParams, obj, 0);

    if warning
        Cost = 1e6;
    else
        data_spline = polyval(p, simulation_time);

        % Making the cost dependent on the rang of points the ODE solves for

        Cost = sum(abs((simulation_x_array - data_spline)));
    end

    fprintf("dt: %.3f seconds\n", toc(timerVal));


end

function [footPos, time, isWarning]= UnifiedModelLight(newFreeParams, obj, lastRun)
    global Fn
    Fn = 1;

    global isSliding
    isSliding = 0;

    %% Free Params used for Model Tunning
    freeParams.finWidth = newFreeParams(1);
    freeParams.muk = newFreeParams(2);
    freeParams.kp_ang = newFreeParams(3);
    freeParams.kd_ang = newFreeParams(4);
    freeParams.mus = newFreeParams(5);

    %% Trajectory sent to the robot

    tstart = tic;

    originalWarningState = warning('query', 'MATLAB:ode23s:IntegrationTolNotMet');

    % Convert the specific warning to an error
    warning('error', 'MATLAB:ode23s:IntegrationTolNotMet');
    try
        if ~lastRun
            [t, q] = ode23s(@(t, q) odefun_unifiedstance(t,q, freeParams, obj), obj.ODEVariables.tspan, obj.ODEVariables.q0, odeset('RelTol', 5e-2, 'Events', @(t, q) swim_event_func(t, q, tstart, Fn)));
        else
            [t, q] = ode23s(@(t, q) odefun_unifiedstance(t,q, freeParams, obj), obj.ODEVariables.tspan, obj.ODEVariables.q0, odeset('RelTol', 5e-2,'Events', @(t, q) swim_event_func_no_time(t, q, tstart, Fn)));
        end
        footPos = q(:,5);
        time = t;
        isWarning = 0;

    catch ME
        disp(['Caught an error (originally a warning): ', ME.message]);
        footPos = [];
        time = [];
        isWarning = 1;

    end

    warning(originalWarningState.state, 'MATLAB:ode23s:IntegrationTolNotMet');

end

function [dq] = odefun_unifiedstance(t,q,freeParams, obj)

    global Fn
    global isSliding

    l = q(1); dl = q(2);
    qB = q(3); dqB = q(4);
    x = q(5); dx = q(6);


    ldes = ldesFunc(t,obj); % desired leg length
    qBdes = qBdesFunc(t,obj); % desired leg angle from horizontal
    dldes = dldesFunc(t,obj); % desired rates
    dqBdes = dqBdesFunc(t,obj);

    Tau = freeParams.kp_ang * (qBdes - q(3)) +  freeParams.kd_ang * (dqBdes - q(4));

    % Compute Fluid Forces (drag forces on hip h, drag forces on leg l, torque
    % due to water
    [Fluid_forces] = getFluidForcesUnified(q, freeParams, obj);

    %% Apply switching logic
    % isSliding = 1;
    if isSliding == 0   % sticking
        % Force of friction is unknown and d2x = 0
        % Use stick dynamics

        Maug = M_aug_func_stick([l; qB]);
        fside = f_func_stick([l; qB; Tau; ldes],[dl; dqB],Fluid_forces);

        ddq_aug_stick = Maug^-1*fside; % [d2l d2qB Ff Fn]

        d2l = ddq_aug_stick(1);
        d2qB = ddq_aug_stick(2);
        d2x = 0; % forced to zero during sticktion

        Ff = ddq_aug_stick(3); % friction (solved for)
        Fn = ddq_aug_stick(4); % normal

        if(  abs(Ff) > freeParams.mus*abs(Fn) )
            isSliding = 1;
        end


        % if(  abs(Ff) > realmax ) % inifinite friction
        %     isSliding = 1;
        % end


    elseif isSliding == 1      %sliding
        % Force of friction is specified and d2x is unknown
        % Use continuous friction
        Maug = M_aug_func_slide([l; qB],[dx],[freeParams.muk]);
        fside = f_func_slide([l; qB; Tau; ldes],[dl; dqB],Fluid_forces);

        ddq_aug_slide = Maug^-1*fside; % ddq_aug = [ddl ddqB ddx Fn];

        d2l = ddq_aug_slide(1);
        d2qB = ddq_aug_slide(2);
        d2x = ddq_aug_slide(3);
        Fn = ddq_aug_slide(4);


        footVel = dx;
        Ff = -freeParams.muk*dx*abs(Fn)/(abs(dx) + obj.SimulationInfo.params.epsilonV);  % specified friction


        if ( ( abs(footVel) < obj.SimulationInfo.params.epsilonV ) && ( abs(Ff) < freeParams.mus*abs(Fn) ) )
            isSliding = 0;
        end



    end

    dq1 = dl; dq2 = d2l;
    dq3 = dqB; dq4 = d2qB;
    dq5 = dx;  dq6 = d2x;

    dq = [dq1; dq2; dq3; dq4; dq5; dq6];



end


function [Fluid_forces] = getFluidForcesUnified(q,freeParams, obj)

    bx_hat = [cos(q(3)); sin(q(3)); 0];
    by_hat = [-sin(q(3)); cos(q(3));0];

    l = q(1); dl = q(2);  % leg length and rate
    qB = q(3); dqB = q(4); % leg angle and rate from ground
    x = q(5); dx = q(6);   % foot position and vel

    wB = freeParams.finWidth;

    Fdrag = [0;0;0]; % drag force on leg
    Torque_water = [0;0;0]; % torque of drag force about foot;

    numForces = numel(obj.SimulationInfo.params.ss_tmp);
    deltaLegDrags = zeros(numForces,3);
    iter = 1;
    for ss = obj.SimulationInfo.params.ss_tmp % integrate over fin

        %vel_p = [dx;0;0] + dqB*ss*by_hat;
        vel_p = [dx;0;0] + dqB*ss*by_hat; % velocity of point p. Located at distance ss from the foot.
        vel_hat = vel_p/norm(vel_p + obj.SimulationInfo.params.epsilonV); % unit vector along velocity at point p

        lambda_hat = cross(vel_hat,[0;0;1]); % unit vector perpendicular to velocity of p

        segment_len_projection = abs( dot(obj.SimulationInfo.params.delta_s*bx_hat, lambda_hat) ) ; % length of projection of ds onto lamba

        Frontal_area = segment_len_projection*wB;

        Fdrag_segment = -0.5*obj.SimulationInfo.params.rho*obj.SimulationInfo.params.Cd_leg*Frontal_area*vel_p*norm(vel_p); % segment drag contribution.
        TorqueDrag_segment = cross(ss*bx_hat,Fdrag_segment); % about foot

        Fdrag = Fdrag + Fdrag_segment;
        Torque_water = Torque_water + TorqueDrag_segment;

        deltaLegDrags(iter,:) = [Fdrag_segment(1) Fdrag_segment(2) ss];
        iter = iter + 1;
    end

    Torque_water_Bcm = Torque_water - cross(0.5*freeParams.finWidth*bx_hat,Fdrag); % torque of water about cm of fin.

    vel_H = q(2)*bx_hat + q(1)*q(4)*by_hat + q(6)*[1;0;0]; % velocity of the hip

    Fdrag_hip = -0.5*obj.SimulationInfo.params.rho*pi*obj.SimulationInfo.params.Cd_hip*obj.SimulationInfo.params.R_2*vel_H*norm(vel_H);

    Fluid_forces = [Fdrag_hip(1); Fdrag_hip(2); Fdrag_segment(1); Fdrag_segment(2); obj.SimulationInfo.params.Fbuoy_hip_y; Torque_water_Bcm(3)];

end

function ldes = ldesFunc(t,obj)
    idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
    if(idx==0)
        idx = 1;
    end
    xdes = obj.ODEVariables.traj(idx,1);
    zdes = obj.ODEVariables.traj(idx,2);
    ldes = sqrt(xdes^2+zdes^2);
end

function qBdes = qBdesFunc(t,obj)
    idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
    if(idx==0)
        idx = 1;
    end
    xdes = obj.ODEVariables.traj(idx,1);
    zdes = obj.ODEVariables.traj(idx,2);
    qBdes = atan2(zdes,xdes) + pi;
end

function dldes = dldesFunc(t,obj)
    idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
    if(idx==0)
        idx = 1;
    end
    xdes = obj.ODEVariables.traj(idx,1);
    zdes = obj.ODEVariables.traj(idx,2);
    ldes = sqrt(xdes^2+zdes^2);

    xdes1 = obj.ODEVariables.traj(idx+1,1);
    zdes1 = obj.ODEVariables.traj(idx+1,2);
    ldes1 = sqrt(xdes1^2+zdes1^2);

    dt = 1/obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));

    dldes = (ldes1-ldes)/dt;

end

function dqBdes = dqBdesFunc(t,obj)
    idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
    if(idx==0)
        idx = 1;
    end
    xdes = obj.ODEVariables.traj(idx,1);
    zdes = obj.ODEVariables.traj(idx,2);
    qBdes = atan2(zdes,xdes);

    xdes1 = obj.ODEVariables.traj(idx+1,1);
    zdes1 = obj.ODEVariables.traj(idx+1,2);
    qBdes1 = atan2(zdes1,xdes1);

    dt = 1/obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));
    dqBdes = (qBdes1-qBdes)/dt;
end

% Event function to detect liftoff
function [value,isterminal,direction] = swim_event_func(t,q, tstart, Fnormal)

    value(1) = Fnormal;
    value(2) = toc(tstart) < 60;
    isterminal = true(size(value));  % Stop integration when event occurs
    direction = -1;  % Detect when Fn crosses zero from positive to negative
end
function [value,isterminal,direction] = swim_event_func_no_time(t,q, tstart, Fnormal)

    value(1) = Fnormal;
    isterminal = true(size(value));  % Stop integration when event occurs
    direction = -1;  % Detect when Fn crosses zero from positive to negative
end


%% Params only
function obj = getParams()
    obj.SimulationInfo.params.g = 9.8;

    obj.SimulationInfo.params.mH = 18.8; %  % Hip mass (two legs in stance)
    obj.SimulationInfo.params.mQ = 23.8/1000; % Foot mass
    obj.SimulationInfo.params.k =  8003; %13592;     % leg stifness (N/m)


    obj.SimulationInfo.params.rho = 1000;
    obj.SimulationInfo.params.R = 0.1568;% 0.1970; % Radius of spherical hip used to match buoyancy
    obj.SimulationInfo.params.R_2 = obj.SimulationInfo.params.R^2;


    obj.SimulationInfo.params.finLen = 10/100;
    obj.SimulationInfo.params.legWidth = 2/10; % leg width

    obj.SimulationInfo.params.epsilonV = 1E-4;
    obj.SimulationInfo.params.Cd_hip = 0.4484;
    obj.SimulationInfo.params.Cd_leg = 2.8118; % drag coefficient leg segment with fin

    obj.SimulationInfo.freq = 2.5; %Hz
    obj.SimulationInfo.stance_duration = 0.16;
    obj.SimulationInfo.vx0_hip = 0.2;
    obj.SimulationInfo.vy0_hip = -0.0; % initial velocity of hip at stance m/s.
    % Should vary with frequency


    obj.SimulationInfo.init_traj = readmatrix("walk_test_2.txt");

    %a Trajectory sent to the robot
    x = obj.SimulationInfo.init_traj(1:500,5);
    z = obj.SimulationInfo.init_traj(1:500,7);

    obj.ODEVariables.traj = [x,z];


    % Precompute trajectory data and time vector
    obj.ODEVariables.t_traj = linspace(0, obj.SimulationInfo.stance_duration, length(obj.ODEVariables.traj(:, 1)));
    obj.ODEVariables.l_traj = sqrt(obj.ODEVariables.traj(:, 1).^2 + obj.ODEVariables.traj(:, 2).^2);
    obj.ODEVariables.qB_traj = atan2(obj.ODEVariables.traj(:, 2), obj.ODEVariables.traj(:, 1)) + pi;

    %Precompute inverse of M matrix
    l0 = ldesFunc(0,obj);
    qB0 = qBdesFunc(0,obj);
    M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
    obj.ODEVariables.Minv = M\eye(2);

    M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
    tmp = M\[obj.SimulationInfo.vx0_hip; obj.SimulationInfo.vy0_hip];

    dl0 = tmp(1); dqB0 = tmp(2);

    x0 = 0;
    dx0 = 0; % initial foot position and velocity

    obj.ODEVariables.q0 = [l0 dl0 qB0 dqB0 x0 dx0];


    obj.SimulationInfo.params.delta_s = 0.01;
    obj.SimulationInfo.params.ss_tmp = 0:obj.SimulationInfo.params.delta_s:obj.SimulationInfo.params.finLen; %iterate over fin
    obj.SimulationInfo.params.numForces = numel(obj.SimulationInfo.params.ss_tmp);
    obj.SimulationInfo.params.deltaLegDrags = zeros(obj.SimulationInfo.params.numForces,3);
    obj.SimulationInfo.params.Vol_hip = (4/3)*pi*obj.SimulationInfo.params.R^3;
    obj.SimulationInfo.params.Fbuoy_hip_y = obj.SimulationInfo.params.rho*obj.SimulationInfo.params.Vol_hip*obj.SimulationInfo.params.g;


    obj.PSOInfo.LB= [0 0.1 10 1 0.1];
    obj.PSOInfo.UB= [2 4 80 10 3];
    init_points = ones([8, 5]) .* [8.18/100 1.0 73 7.3 1.5] ;
    obj.PSOInfo.options = optimoptions('particleswarm', 'SwarmSize', 8, 'UseParallel', true, 'InitialPoints', init_points, 'MaxIterations', 1);
    obj.PSOInfo.nvars = 5;

    obj.SimulationInfo.params.dt_inv = obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));


end

function f = f_func_stick(in1,in2,in3)
    %F_FUNC_STICK
    %    F = F_FUNC_STICK(IN1,IN2,IN3)

    %    This function was generated by the Symbolic Math Toolbox version 23.2.
    %    24-Feb-2025 14:50:14

    Fbh_y = in3(5,:);
    Fdh_x = in3(1,:);
    Fdh_y = in3(2,:);
    Fdl_x = in3(3,:);
    Fdl_y = in3(4,:);
    T_w = in3(6,:);
    Tau = in1(3,:);
    dl = in2(1,:);
    dqB = in2(2,:);
    l = in1(1,:);
    ldes = in1(4,:);
    qB = in1(2,:);
    t2 = cos(qB);
    t3 = sin(qB);
    t4 = dqB.^2;
    f = [Fdh_x+Fdl_x+dl.*dqB.*t3.*(1.88e+2./5.0)+l.*t2.*t4.*(9.4e+1./5.0);Fbh_y+Fdh_y+Fdl_y-dl.*dqB.*t2.*(1.88e+2./5.0)+l.*t3.*t4.*(9.4e+1./5.0)-1.8447324e+2;l.*-8.003e+3+ldes.*8.003e+3+t3.*(Fbh_y+Fdh_y+Fdl_y-1.8424e+2)+l.*t4.*(9.4e+1./5.0)+t2.*(Fdh_x+Fdl_x);T_w+Tau+l.*t2.*(Fbh_y+Fdh_y-1.8424e+2)-Fdh_x.*l.*t3-Fdl_x.*l.*t3.*5.0e-1+Fdl_y.*l.*t2.*5.0e-1-dl.*dqB.*l.*(1.88e+2./5.0)];
end

function Maug = M_aug_func_stick(in1)
    %M_aug_func_stick
    %    Maug = M_aug_func_stick(IN1)

    %    This function was generated by the Symbolic Math Toolbox version 23.2.
    %    24-Feb-2025 14:50:14

    l = in1(1,:);
    qB = in1(2,:);
    t2 = cos(qB);
    t3 = sin(qB);
    Maug = reshape([t2.*(9.4e+1./5.0),t3.*(9.4e+1./5.0),9.4e+1./5.0,0.0,l.*t3.*(-9.4e+1./5.0),l.*t2.*(9.4e+1./5.0),0.0,l.^2.*(9.4e+1./5.0),-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0],[4,4]);
end

function f = f_func_slide(in1,in2,in3)
    %F_FUNC_SLIDE
    %    F = F_FUNC_SLIDE(IN1,IN2,IN3)

    %    This function was generated by the Symbolic Math Toolbox version 23.2.
    %    24-Feb-2025 14:56:05

    Fbh_y = in3(5,:);
    Fdh_x = in3(1,:);
    Fdh_y = in3(2,:);
    Fdl_x = in3(3,:);
    Fdl_y = in3(4,:);
    T_w = in3(6,:);
    Tau = in1(3,:);
    dl = in2(1,:);
    dqB = in2(2,:);
    l = in1(1,:);
    ldes = in1(4,:);
    qB = in1(2,:);
    t2 = cos(qB);
    t3 = sin(qB);
    t4 = dqB.^2;
    f = [Fdh_x+Fdl_x+dl.*dqB.*t3.*(1.88e+2./5.0)+l.*t2.*t4.*(9.4e+1./5.0);Fbh_y+Fdh_y+Fdl_y-dl.*dqB.*t2.*(1.88e+2./5.0)+l.*t3.*t4.*(9.4e+1./5.0)-1.8447324e+2;l.*-8.003e+3+ldes.*8.003e+3+t3.*(Fbh_y+Fdh_y+Fdl_y-1.8424e+2)+l.*t4.*(9.4e+1./5.0)+t2.*(Fdh_x+Fdl_x);T_w+Tau+l.*t2.*(Fbh_y+Fdh_y-1.8424e+2)-Fdh_x.*l.*t3-Fdl_x.*l.*t3.*5.0e-1+Fdl_y.*l.*t2.*5.0e-1-dl.*dqB.*l.*(1.88e+2./5.0)];
end

function Maug = M_aug_func_slide(in1,dx,muk)
    %M_aug_func_slide
    %    Maug = M_aug_func_slide(IN1,DX,MUK)

    %    This function was generated by the Symbolic Math Toolbox version 23.2.
    %    24-Feb-2025 14:56:04

    l = in1(1,:);
    qB = in1(2,:);
    t2 = cos(qB);
    t3 = sin(qB);
    t4 = t2.*(9.4e+1./5.0);
    t5 = l.*t3.*(9.4e+1./5.0);
    t6 = -t5;
    Maug = reshape([t4,t3.*(9.4e+1./5.0),9.4e+1./5.0,0.0,t6,l.*t4,0.0,l.^2.*(9.4e+1./5.0),1.88238e+1,0.0,t4,t6,(dx.*muk)./(abs(dx)+1.0e-5),-1.0,0.0,0.0],[4,4]);
end
