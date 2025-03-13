
%{

Make sure LB is fin width!

%}
clc; clear; close all; format compact
% parpool

load("T_Results.mat");


obj = getParams();

t_results = t(15);
cur_t_vals = t_results.t(1:t_results.StatsPlottingTrialLength);
cur_x_vals = t_results.MeanXPosition(1:t_results.StatsPlottingTrialLength);

p = polyfit(cur_t_vals, cur_x_vals, 13);


% initial conditions
l0 = ldesFunc(0, obj);
qB0 = qBdesFunc(0, obj);

% Should vary with frequency

%% Run PSO

costFunctionHandle = @(freeParams) ModelSimulationCost(freeParams, obj, p);

disp("Starting PSO...");
psoTimer = tic;
[OptimizedState, FVAL] = particleswarm(costFunctionHandle, obj.PSOInfo.nvars, obj.PSOInfo.LB, obj.PSOInfo.UB, obj.PSOInfo.options);
fprintf("\n\nPSO Completed in %.3f seconds", toc(psoTimer));

disp(OptimizedState);

[footPos, time] = UnifiedModelLight(OptimizedState, obj);

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
    [simulation_x_array, simulation_time, warning] = UnifiedModelLight(freeParams, obj);


    if warning
        Cost = 1e6;
    else
        data_spline = polyval(p, simulation_time);

        % Making the cost dependent on the rang of points the ODE solves for

        Cost = (norm(simulation_x_array - data_spline)^2) * abs(2/numel(simulation_time));
    end

    fprintf("dt: %.3f seconds\n", toc(timerVal));


end

function [footPos, time, isWarning]= UnifiedModelLight(newFreeParams, obj)
    global Fn
    Fn = 1;
    %
    % global isSliding
    % isSliding = 1;

    %% Free Params used for Model Tunning
    freeParams.finWidth = newFreeParams(1);
    freeParams.muk = newFreeParams(2);
    freeParams.kp_ang = newFreeParams(3);
    freeParams.kd_ang = newFreeParams(4);
    freeParams.mus = newFreeParams(5);
    obj.SimulationInfo.params.Cd_hip = newFreeParams(6);
    obj.SimulationInfo.params.Cd_leg = newFreeParams(7);


    % freeParams.kd_ang = 0.1*newFreeParams(3);
    % freeParams.mus = 1.3*newFreeParams(2);


    %% Trajectory sent to the robot

    tstart = tic;

    originalWarningState = warning('query', 'MATLAB:ode15s:IntegrationTolNotMet');

    % Convert the specific warning to an error
    warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
    try
        [t, q] = ode15s(@(t, q) odefun_unifiedstance(t,q, freeParams, obj), obj.ODEVariables.tspan, obj.ODEVariables.q0, odeset('Events', @(t, q) swim_event_func(t, q, tstart, Fn)));
        footPos = q(:,5);
        time = t;
        isWarning = 0;

    catch ME
        disp(['Caught an error (originally a warning): ', ME.message]);
        footPos = [];
        time = [];
        isWarning = 1;

    end

    warning(originalWarningState.state, 'MATLAB:ode15s:IntegrationTolNotMet');

end

function [dq] = odefun_unifiedstance(t,q,freeParams, obj)

    global Fn


    % Replacing ldesFunc
    % idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
    % idx = idx + (idx==0);
    % ldes = sqrt(obj.ODEVariables.traj(idx,1)^2+obj.ODEVariables.traj(idx,2)^2);

    try
        ldes = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.l_traj, t, 'linear', 'extrap');

        % Replacing qBdesFunc
        % idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
        % idx = idx + (idx==0);
        % qBdes = atan2(obj.ODEVariables.traj(idx,2),obj.ODEVariables.traj(idx,1)) + pi;

        % Replacing dqBdesFunc
        % idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
        % idx = idx + (idx==0);
        % qBdes = atan2(obj.ODEVariables.traj(idx,2),obj.ODEVariables.traj(idx,1));
        % qBdes = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.qB_traj, t, 'linear', 'extrap');

        % Back to normal
        % qBdes1 = atan2(obj.ODEVariables.traj(idx+1,2),obj.ODEVariables.traj(idx+1,1));

        dt = 1e-5; % Small time step for numerical derivative
        qBdes = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.qB_traj, t, 'linear', 'extrap');
        qBdes1 = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.qB_traj, t + dt, 'linear', 'extrap');
        dqBdes = (qBdes1 - qBdes) / dt;

        Tau = freeParams.kp_ang * (qBdes - q(3)) +  freeParams.kd_ang * (dqBdes - q(4));

        % Compute Fluid Forces (drag forces on hip h, drag forces on leg l, torque
        % due to water
        [Fluid_forces] = getFluidForcesUnified(q, freeParams, obj);



        %% Apply switching logic
        % isSliding = 1;
        % switch(isSliding)
        %     case 0      % sticking
        % Force of friction is unknown and d2x = 0
        % Use stick dynamics

        % Maug = M_aug_func_stick([q(1); q(3)]);
        % fside = f_func_stick([q(1); q(3); Tau; ldes],[q(2); q(4)],Fluid_forces);
        %
        % ddq_aug_stick = Maug^-1*fside; % [d2l d2qB Ff Fn]
        %
        % Fn = ddq_aug_stick(4); % normal
        %
        % dq = [q(2); ddq_aug_stick(1); q(4); ddq_aug_stick(2); q(6); 0];
        %
        % if(  abs(ddq_aug_stick(3)) > freeParams.mus*abs(Fn) )
        %     isSliding = 1;
        % end


        % case 1      %sliding
        % Force of friction is specified and d2x is unknown
        % Use continuous friction
        Maug = M_aug_func_slide([q(1); q(3)],[q(6)],[freeParams.muk]);
        fside = f_func_slide([q(1); q(3); Tau; ldes],[q(2); q(4)],Fluid_forces);

        ddq_aug_slide = Maug^-1*fside; % ddq_aug = [ddl ddqB ddx Fn];

        Fn = ddq_aug_slide(4);


        % footVel = q(6);
        % Ff = -freeParams.muk*q(6)*abs(Fn)/(abs(q(6)) + obj.SimulationInfo.params.epsilonV);  % specified friction

        dq = [q(2); ddq_aug_slide(1); q(4); ddq_aug_slide(2); q(6); ddq_aug_slide(3)];

        % if ( ( abs(footVel) < obj.SimulationInfo.params.epsilonV ) && ( abs(Ff) < freeParams.mus*abs(Fn) ) )
        %     isSliding = 0;
        % end

    catch ME

        warningCallback(ME.message);
        dq = zeros(size(q)); % Return zeros if error occurs
    end


end


function [Fluid_forces] = getFluidForcesUnified(q,freeParams, obj)

    bx_hat = [cos(q(3)); sin(q(3)); 0];
    by_hat = [-sin(q(3)); cos(q(3));0];

    Fdrag = [0;0;0]; % drag force on leg
    Torque_water = [0;0;0]; % torque of drag force about foot;

    % vel_p = [q(6);0;0] + q(4)*obj.SimulationInfo.params.ss_tmp.*by_hat
    % vel_hat = vel_p/norm(vel_p + obj.SimulationInfo.params.epsilonV);
    % Frontal_area = abs( dot(obj.SimulationInfo.params.delta_s*[bx_hat,bx_hat,bx_hat], cross(vel_hat,[0,0,0;0,0,0;1,1,1])) )*freeParams.finWidth
    % Fdrag_segment = -0.5*obj.SimulationInfo.params.rho*obj.SimulationInfo.params.Cd_leg*Frontal_area*vel_p*norm(vel_p);

    for ss = obj.SimulationInfo.params.ss_tmp % integrate over fin

        vel_p = [q(6);0;0] + q(4)*ss*by_hat; % velocity of point p. Located at distance ss from the foot.
        vel_hat = vel_p/norm(vel_p + obj.SimulationInfo.params.epsilonV); % unit vector along velocity at point p

        Frontal_area = abs( dot(obj.SimulationInfo.params.delta_s*bx_hat, cross(vel_hat,[0;0;1])) )*freeParams.finWidth;

        Fdrag_segment = -0.5*obj.SimulationInfo.params.rho*obj.SimulationInfo.params.Cd_leg*Frontal_area*vel_p*norm(vel_p); % segment drag contribution.

        Fdrag = Fdrag + Fdrag_segment;
        Torque_water = Torque_water + cross(ss*bx_hat,Fdrag_segment);

    end

    Torque_water_Bcm = Torque_water - cross(0.5*freeParams.finWidth*bx_hat,Fdrag); % torque of water about cm of fin.


    vel_H = q(2)*bx_hat + q(1)*q(4)*by_hat + q(6)*[1;0;0]; % velocity of the hip

    Fdrag_hip = -0.5*obj.SimulationInfo.params.rho*pi*obj.SimulationInfo.params.Cd_hip*obj.SimulationInfo.params.R_2*vel_H*norm(vel_H);

    Fluid_forces = [Fdrag_hip(1); Fdrag_hip(2); Fdrag_segment(1); Fdrag_segment(2); obj.SimulationInfo.params.Fbuoy_hip_y; Torque_water_Bcm(3)];

end

% function [Fluid_forces] = getFluidForcesUnified(q, freeParams, obj)
%     % Calculate bx_hat and by_hat
%     cos_q3 = cos(q(3));
%     sin_q3 = sin(q(3));
%     bx_hat = [cos_q3; sin_q3; 0];
%     by_hat = [-sin_q3; cos_q3; 0];
%
%     % Vectorized vel_p calculation
%     vel_p = [q(6) + q(4) * obj.SimulationInfo.params.ss_tmp * by_hat(1); zeros(size(obj.SimulationInfo.params.ss_tmp)); q(4) * obj.SimulationInfo.params.ss_tmp * by_hat(2)];
%
%     % Vectorized norm and vel_hat calculation
%     vel_norm = sqrt(sum(vel_p.^2, 1)) + obj.SimulationInfo.params.epsilonV;
%     vel_hat = vel_p ./ vel_norm;
%
%     % Vectorized Frontal_area calculation
%     cross_vec = cross(vel_hat, repmat([0; 0; 1], 1, length(obj.SimulationInfo.params.ss_tmp)));
%     Frontal_area = abs(obj.SimulationInfo.params.delta_s * sum(bx_hat .* cross_vec, 1)) * freeParams.finWidth;
%
%     % Vectorized Fdrag_segment calculation
%     Fdrag_segment = -0.5 * obj.SimulationInfo.params.rho * obj.SimulationInfo.params.Cd_leg * Frontal_area .* vel_p .* vel_norm;
%
%     % Vectorized Torque_water calculation
%     cross_torque = cross(repmat(obj.SimulationInfo.params.ss_tmp, 3, 1) .* repmat(bx_hat, 1, length(obj.SimulationInfo.params.ss_tmp)), Fdrag_segment);
%     Torque_water = sum(cross_torque, 2);
%
%     % Torque_water_Bcm calculation
%     Fdrag_total = sum(Fdrag_segment, 2);
%     Torque_water_Bcm = Torque_water - cross(0.5 * freeParams.finWidth * bx_hat, Fdrag_total);
%
%     % vel_H calculation
%     vel_H = q(2) * bx_hat + q(1) * q(4) * by_hat + [q(6); 0; 0];
%
%     % Fdrag_hip calculation
%     vel_H_norm = norm(vel_H);
%     Fdrag_hip = -0.5 * obj.SimulationInfo.params.rho * pi * obj.SimulationInfo.params.Cd_hip * obj.SimulationInfo.params.R^2 * vel_H * vel_H_norm;
%
%     % Fluid_forces output
%     Fluid_forces = [Fdrag_hip(1); Fdrag_hip(2); reshape(Fdrag_segment(1:2, :), [], 1); obj.SimulationInfo.params.Fbuoy_hip_y; Torque_water_Bcm(3)];
% end
% function ldes = ldesFunc(t, obj)
%     idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
%
%     idx = idx + (idx==0);
%
%     ldes = sqrt(obj.ODEVariables.traj(idx,1)^2+obj.ODEVariables.traj(idx,2)^2);
% end
%
% function qBdes = qBdesFunc(t,obj)
%     idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
%     idx = idx + (idx==0);
%
%     qBdes = atan2(obj.ODEVariables.traj(idx,2),obj.ODEVariables.traj(idx,1)) + pi;
% end
%
% function dldes = dldesFunc(t, obj)
%     idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
%
%     idx = idx + (idx==0);
%
%     ldes = sqrt(obj.ODEVariables.traj(idx,1)^2+obj.ODEVariables.traj(idx,2)^2);
%
%     ldes1 = sqrt(obj.ODEVariables.traj(idx+1,1)^2+obj.ODEVariables.traj(idx+1,2)^2);
%
%     dt_inv = obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));
%
%     dldes = (ldes1-ldes)*dt_inv;
%
% end
%
%
% function dqBdes = dqBdesFunc(t, obj)
%     idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
%
%     idx = idx + (idx==0);
%
%     qBdes = atan2(obj.ODEVariables.traj(idx,2),obj.ODEVariables.traj(idx,1));
%
%     qBdes1 = atan2(obj.ODEVariables.traj(idx+1,2),obj.ODEVariables.traj(idx+1,1));
%
%     dt_inv = obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));
%     dqBdes = (qBdes1-qBdes)*dt_inv;
% end

function ldes = ldesFunc(t, obj)
    ldes = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.l_traj, t, 'linear', 'extrap');
end

function qBdes = qBdesFunc(t, obj)
    qBdes = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.qB_traj, t, 'linear', 'extrap');
end

function dldes = dldesFunc(t, obj)
    dt = 1e-6; % Small time step for numerical derivative
    ldes1 = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.l_traj, t, 'linear', 'extrap');
    ldes2 = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.l_traj, t + dt, 'linear', 'extrap');
    dldes = (ldes2 - ldes1) / dt;
end

function dqBdes = dqBdesFunc(t, obj)
    dt = 1e-6; % Small time step for numerical derivative
    qBdes1 = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.qB_traj, t, 'linear', 'extrap');
    qBdes2 = interp1(obj.ODEVariables.t_traj, obj.ODEVariables.qB_traj, t + dt, 'linear', 'extrap');
    dqBdes = (qBdes2 - qBdes1) / dt;
end

function Maug = M_aug_func_slide(in1,dx,muk)
    Maug = reshape([cos(in1(2,:)).*(9.4e+1./5.0),sin(in1(2,:)).*(9.4e+1./5.0),9.4e+1./5.0,0.0,(-(in1(1,:).*sin(in1(2,:)).*(9.4e+1./5.0))),in1(1,:).*cos(in1(2,:)).*(9.4e+1./5.0),0.0,in1(1,:).^2.*(9.4e+1./5.0),1.88238e+1,0.0,cos(in1(2,:)).*(9.4e+1./5.0),(-(in1(1,:).*sin(in1(2,:)).*(9.4e+1./5.0))),(dx.*muk)./(abs(dx)+1.0e-5),-1.0,0.0,0.0],[4,4]);
end

function Maug = M_aug_func_stick(in1)
    Maug = reshape([cos(in1(2,:)).*(9.4e+1./5.0),sin(in1(2,:)).*(9.4e+1./5.0),9.4e+1./5.0,0.0,in1(1,:).*sin(in1(2,:)).*(-9.4e+1./5.0),in1(1,:).*cos(in1(2,:)).*(9.4e+1./5.0),0.0,in1(1,:).^2.*(9.4e+1./5.0),-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0],[4,4]);
end

function f = f_func_slide(in1,in2,in3)
    f = [in3(1,:)+in3(3,:)+in2(1,:).*in2(2,:).*sin(in1(2,:)).*(1.88e+2./5.0)+in1(1,:).*cos(in1(2,:)).*in2(2,:).^2.*(9.4e+1./5.0);in3(5,:)+in3(2,:)+in3(4,:)-in2(1,:).*in2(2,:).*cos(in1(2,:)).*(1.88e+2./5.0)+in1(1,:).*sin(in1(2,:)).*in2(2,:).^2.*(9.4e+1./5.0)-1.8447324e+2;in1(1,:).*-8.003e+3+in1(4,:).*8.003e+3+sin(in1(2,:)).*(in3(5,:)+in3(2,:)+in3(4,:)-1.8424e+2)+in1(1,:).*in2(2,:).^2.*(9.4e+1./5.0)+cos(in1(2,:)).*(in3(1,:)+in3(3,:));in3(6,:)+in1(3,:)+in1(1,:).*cos(in1(2,:)).*(in3(5,:)+in3(2,:)-1.8424e+2)-in3(1,:).*in1(1,:).*sin(in1(2,:))-in3(3,:).*in1(1,:).*sin(in1(2,:)).*5.0e-1+in3(4,:).*in1(1,:).*cos(in1(2,:)).*5.0e-1-in2(1,:).*in2(2,:).*in1(1,:).*(1.88e+2./5.0)];
end

function f = f_func_stick(in1,in2,in3)
    f = [in3(1,:)+in3(3,:)+in2(1,:).*in2(2,:).*sin(in1(2,:)).*(1.88e+2./5.0)+in1(1,:).*cos(in1(2,:)).*in2(2,:).^2.*(9.4e+1./5.0);in3(5,:)+in3(2,:)+in3(4,:)-in2(1,:).*in2(2,:).*cos(in1(2,:)).*(1.88e+2./5.0)+in1(1,:).*sin(in1(2,:)).*in2(2,:).^2.*(9.4e+1./5.0)-1.8447324e+2;in1(1,:).*-8.003e+3+in1(4,:).*8.003e+3+sin(in1(2,:)).*(in3(5,:)+in3(2,:)+in3(4,:)-1.8424e+2)+in1(1,:).*in2(2,:).^2.*(9.4e+1./5.0)+cos(in1(2,:)).*(in3(1,:)+in3(3,:));in3(6,:)+in1(3,:)+in1(1,:).*cos(in1(2,:)).*(in3(5,:)+in3(2,:)-1.8424e+2)-in3(1,:).*in1(1,:).*sin(in1(2,:))-in3(3,:).*in1(1,:).*sin(in1(2,:)).*5.0e-1+in3(4,:).*in1(1,:).*cos(in1(2,:)).*5.0e-1-in2(1,:).*in2(2,:).*in1(1,:).*(1.88e+2./5.0)];
end

% Event function to detect liftoff
function [value,isterminal,direction] = swim_event_func(t,q,tstart, Fnormal)
    % Stop when Fn = 0
    value(1) = Fnormal;
    value(2) = toc(tstart) < 30;
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


    obj.ODEVariables.tspan = [0 obj.SimulationInfo.stance_duration];

    obj.ODEVariables.q0 = [l0 dl0 qB0 dqB0 x0 dx0];


    obj.SimulationInfo.params.delta_s = 0.02;
    obj.SimulationInfo.params.ss_tmp = 0:obj.SimulationInfo.params.delta_s:obj.SimulationInfo.params.finLen; %iterate over fin
    obj.SimulationInfo.params.numForces = numel(obj.SimulationInfo.params.ss_tmp);
    obj.SimulationInfo.params.deltaLegDrags = zeros(obj.SimulationInfo.params.numForces,3);
    obj.SimulationInfo.params.Vol_hip = (4/3)*pi*obj.SimulationInfo.params.R^3;
    obj.SimulationInfo.params.Fbuoy_hip_y = obj.SimulationInfo.params.rho*obj.SimulationInfo.params.Vol_hip*obj.SimulationInfo.params.g;


    obj.PSOInfo.LB= [5 0.1 10 0.1 0.1 0.1 0.1];
    obj.PSOInfo.UB= [30 10 60 3 3 100];
    obj.PSOInfo.options = optimoptions('particleswarm', 'SwarmSize', 300, 'UseParallel', true);
    obj.PSOInfo.nvars = 7;

    obj.SimulationInfo.params.dt_inv = obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));


end




