% Camilo Ordonez - 2025
% EDITED: Jack Vranicar - 2025

%% Formatting
clc
clear
close all

addpath("ModelFunctionsFiles");
addpath("ModelFunctionsFiles\autoSlide\")
addpath("ModelFunctionsFiles\autoStick\")

%% Loading trajectory
traj = readmatrix("walk_test_2.txt");

%% Loading and organizing data from experiment
load("T_Results.mat");
t_results = t(15);
cur_t_vals = t_results.t(1:t_results.StatsPlottingTrialLength);
cur_x_vals = t_results.MeanXPosition(1:t_results.StatsPlottingTrialLength);

p = polyfit(cur_t_vals, cur_x_vals, 13);

%% Setting parameters
params = jv_getParams();


global isSliding;
isSliding = 0;

g = params.SimulationInfo.params.g;
mH = params.SimulationInfo.params.mH; % Hip mass
mQ = params.SimulationInfo.params.mQ; % Foot mass
k = params.SimulationInfo.params.k;   % leg stifness (N/m)
epsilonV = params.SimulationInfo.params.epsilonV;

%% Run PSO

costFunctionHandle = @(freeParams) ModelSimulationCost(freeParams, obj, p);

disp("Starting PSO...");
psoTimer = tic;
[OptimizedState, FVAL] = particleswarm(costFunctionHandle, params.PSOInfo.nvars, params.PSOInfo.LB, params.PSOInfo.UB, params.PSOInfo.options);
fprintf("\n\nPSO Completed in %.3f seconds", toc(psoTimer));

disp(OptimizedState);

[footPos, time] = UnifiedModelLight(OptimizedState, params);

figure
hold on
plot(time, footPos, 'xr');
plot(cur_t_vals, cur_x_vals, 'k');
legend("Best Simulation", "Experimental Data");
%% Creating a cost function for PSO
function [Cost] = ModelSimulationCost(freeParams, params, p)

    timerVal = tic;
    fprintf("Starting a Simulation...");
    % Run the simulation to determine the range of x values
    [simulation_x_array, simulation_time, warning] = UnifiedModelLight(freeParams, params);


    if warning
        Cost = 1e6;
    else
        data_spline = polyval(p, simulation_time);

        % Making the cost dependent on the rang of points the ODE solves for

        Cost = (norm(simulation_x_array - data_spline)^2) * abs(2/numel(simulation_time));
    end

    fprintf("dt: %.3f seconds\n", toc(timerVal));


end

function [footPos, time, isWarning]= UnifiedModelLight(newFreeParams, params)
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
    params.SimulationInfo.params.Cd_hip = newFreeParams(6);
    params.SimulationInfo.params.Cd_leg = newFreeParams(7);


    % freeParams.kd_ang = 0.1*newFreeParams(3);
    % freeParams.mus = 1.3*newFreeParams(2);


    %% Trajectory sent to the robot

    tstart = tic;

    originalWarningState = warning('query', 'MATLAB:ode15s:IntegrationTolNotMet');

    % Convert the specific warning to an error
    warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
    try
        [t, q] = ode15s(@(t, q) jv_odefun_unifiedstance(t,q, freeParams, params), params.ODEVariables.tspan, params.ODEVariables.q0, odeset('Events', @(t, q) swim_event_func(t, q, tstart, Fn)));
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
%%
freq = 2.5; %Hz
stance_duration = 0.16;
vx0_hip = 0.2; vy0_hip = 0.0; % initial velocity of hip at stance m/s.
% Should vary with frequency


%% Free Params used for Model Tunning
freeParams.kp_ang = 73; % controller gains for applied torque on leg
freeParams.kd_ang = freeParams.kp_ang/10;
freeParams.muk = 1.0;
freeParams.mus = 1.5;
freeParams.finWidth = 8.18/100; %m

%% Trajectory sent to the robot
x = traj(1:500,5);
z = traj(1:500,7);

figure(1)

plot(x,z,'o')
hold on
plot(x(1),z(1),'or')

traj = [x,z];




for tt = 0:0.001:stance_duration

    idx = ceil(tt*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);

    hold on
    plot(xdes,zdes,'or')
end


% initial conditions


l0 = ldesFunc(0,traj,freq);
qB0 = qBdesFunc(0,traj,freq);

% Find initial conditions that are consistent with initial hip velocity
%vx_hip  = dx + cos(qB)*dl - l*sin(qB)*dqB;
%vy_hip  = sin(qB)*dl - l*cos(qB)*dqB;

M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
tmp = inv(M)*[vx0_hip;vy0_hip];

dl0 = tmp(1); dqB0 = tmp(2);

x0 = 0;
dx0 = 0; % initial foot position and velocity


tspan = [0 stance_duration];

q0 = [l0 dl0 qB0 dqB0 x0 dx0];

% Set event function
options = odeset('Events', @swim_event_func);

isSliding = 0;
%[t, q] = ode45(@(t, q) odefun_unifiedstance(t,q, traj, freq, freeParams), tspan, q0, options);
[t, q] = ode23s(@(t, q) odefun_unifiedstance(t,q, traj, freq, freeParams), tspan, q0, options);

footPos = q(:,5);

figure
plot(t,footPos)
ylabel('FootPosition(m)')
xlabel('time (s)')

%keyboard()
FPS = 100;
animateUnified(t,q,FPS)


%
%
%
%
%
% % compute Ground reaction forces
% isSliding = 0;
% for iter = 1:numel(t)
%
%     tt = t(iter);
%     qq = q(iter,:);
%
%     l = qq(1); dl = qq(2);
%     qB = qq(3); dqB = qq(4);
%     x = qq(5); dx = qq(6);
%
%     ldes = ldesFunc(tt,traj,freq); % desired leg length
%     qBdes = qBdesFunc(tt,traj,freq); % desired leg angle from horizontal
%
%     dldes = dldesFunc(tt,traj,freq); % desired rates
%     dqBdes = dqBdesFunc(tt,traj,freq);
%
%     Tau = kp_ang * (qBdes - qB) +  kd_ang * (dqBdes - dqB);
%
%     % Compute fluid forces
%     [Fluid_forces, deltaLegDrags] = getFluidForcesUnified(qq);
%
%     %%
%
%     %% Apply switching logic
%
%     switch(isSliding)
%         case 0      % sticking
%         % Force of friction is unknown and d2x = 0
%         % Use stick dynamics
%
%         Maug = M_aug_func_stick([l; qB]);
%         fside = f_func_stick([l; qB; Tau; ldes],[dl; dqB],Fluid_forces);
%
%         ddq_aug_stick = Maug^-1*fside; % [d2l d2qB Ff Fn]
%
%         d2l = ddq_aug_stick(1);
%         d2qB = ddq_aug_stick(2);
%         d2x = 0; % forced to zero during sticktion
%
%         Ff = ddq_aug_stick(3); % friction (solved for)
%         Fn = ddq_aug_stick(4); % normal
%
%         if(  abs(Ff) > mus*abs(Fn) )
%             isSliding = 1;
%             %keyboard()
%         end
%
%
%     case 1      %sliding
%         % Force of friction is specified and d2x is unknown
%         % Use continuous friction
%         Maug = M_aug_func_slide([l; qB],[dx]);
%         fside = f_func_slide([l; qB; Tau; ldes],[dl; dqB],Fluid_forces);
%
%         ddq_aug_slide = Maug^-1*fside; % ddq_aug = [ddl ddqB ddx Fn];
%
%         d2l = ddq_aug_slide(1);
%         d2qB = ddq_aug_slide(2);
%         d2x = ddq_aug_slide(3);
%         Fn = ddq_aug_slide(4);
%
%
%         footVel = dx;
%         Ff = -muk*dx*abs(Fn)/(abs(dx) + epsilonV);  % specified friction
%
%
%         if ( ( abs(footVel) < epsilonV ) && ( abs(Ff) < mus*abs(Fn) ) )
%             isSliding = 0;
%         end
%
%
%
%     end
%
%
%     Friction(iter) = Ff;
%     NormalForce(iter) = Fn;
%
%
%
% end
%
%
%
% figure()
%
% subplot(2,1,1)
%     plot(t,Friction)
%     ylabel('Friction (N)')
%
%  subplot(2,1,2)
%    plot(t,NormalForce)
%    ylabel('Normal Force (N)')


%% My version of GetParams

function obj = jv_getParams()

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
    % l0 = ldesFunc(0,obj);
    % qB0 = qBdesFunc(0,obj);
    % M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
    % obj.ODEVariables.Minv = M\eye(2);

    % M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
    % tmp = M\[obj.SimulationInfo.vx0_hip; obj.SimulationInfo.vy0_hip];
    % 
    % dl0 = tmp(1); dqB0 = tmp(2);
    % 
    % x0 = 0;
    % dx0 = 0; % initial foot position and velocity


    % obj.ODEVariables.tspan = [0 obj.SimulationInfo.stance_duration];
    % 
    % obj.ODEVariables.q0 = [l0 dl0 qB0 dqB0 x0 dx0];


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

function [dq] = jv_odefun_unifiedstance(t,q, traj, freq, freeParams)

    global isSliding;
    global Fnormal;

    l = q(1); dl = q(2);
    qB = q(3); dqB = q(4);
    x = q(5); dx = q(6);


    params = getParams();

    g = params.g;
    mH = params.mH; % Hip mass
    mQ = params.mQ; % Foot mass
    k = params.k;   % leg stifness (N/m)
    epsilonV = params.epsilonV;

    kp_ang = freeParams.kp_ang; % controller gains for applied torque on leg
    kd_ang = freeParams.kd_ang;
    muk = freeParams.muk;
    mus = freeParams.mus;


    try

        ldes = ldesFunc(t,traj,freq); % desired leg length
        qBdes = qBdesFunc(t,traj,freq); % desired leg angle from horizontal

        dldes = dldesFunc(t,traj,freq); % desired rates
        dqBdes = dqBdesFunc(t,traj,freq);

        Tau = kp_ang * (qBdes - qB) +  kd_ang * (dqBdes - dqB);

        % Compute Fluid Forces (drag forces on hip h, drag forces on leg l, torque
        % due to water
        [Fluid_forces, deltaLegDrags] = getFluidForcesUnified(q, freeParams);



        %% Apply switching logic
        isSliding
        switch(isSliding)
            case 0      % sticking
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

                if(  abs(Ff) > mus*abs(Fn) )
                    isSliding = 1;
                end


                % if(  abs(Ff) > realmax ) % inifinite friction
                %     isSliding = 1;
                % end


            case 1      %sliding
                % Force of friction is specified and d2x is unknown
                % Use continuous friction
                Maug = M_aug_func_slide([l; qB],[dx],[muk]);
                fside = f_func_slide([l; qB; Tau; ldes],[dl; dqB],Fluid_forces);

                ddq_aug_slide = Maug^-1*fside; % ddq_aug = [ddl ddqB ddx Fn];

                d2l = ddq_aug_slide(1);
                d2qB = ddq_aug_slide(2);
                d2x = ddq_aug_slide(3);
                Fn = ddq_aug_slide(4);


                footVel = dx;
                Ff = -muk*dx*abs(Fn)/(abs(dx) + epsilonV);  % specified friction


                if ( ( abs(footVel) < epsilonV ) && ( abs(Ff) < mus*abs(Fn) ) )
                    isSliding = 0;
                end



        end

        Fnormal = Fn;

        dq1 = dl; dq2 = d2l;
        dq3 = dqB; dq4 = d2qB;
        dq5 = dx;  dq6 = d2x;

        dq = [dq1; dq2; dq3; dq4; dq5; dq6];
    catch ME
        warningCallback(ME.message);
        dq = zeros(size(q)); % return 0s if error occurs
    end
end