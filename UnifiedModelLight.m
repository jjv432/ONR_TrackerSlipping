function [footPos, time]= UnifiedModelLight(newFreeParams)

    clear footPos;

    persistent init_traj params

    if isempty(init_traj)
        init_traj = readmatrix("walk_test_2.txt");
    end

    if isempty(params)
        params = getParams();
    end

    global isSliding;
    isSliding = 0;



    g = params.g;
    mH = params.mH; % Hip mass
    mQ = params.mQ; % Foot mass
    k = params.k;   % leg stifness (N/m)
    epsilonV = params.epsilonV;

    %%
    freq = 2.5; %Hz
    stance_duration = 0.16;
    vx0_hip = 0.2; vy0_hip = -0.0; % initial velocity of hip at stance m/s.
    % Should vary with frequency


    %% Free Params used for Model Tunning
    freeParams.finWidth = newFreeParams(1);
    freeParams.muk = newFreeParams(2);
    freeParams.kp_ang = newFreeParams(3);
    freeParams.kd_ang = (1/10)*freeParams.kp_ang;
    freeParams.mus = 1.3*freeParams.muk;


    %% Trajectory sent to the robot
    x = init_traj(1:500,5);
    z = init_traj(1:500,7);

    traj = [x,z];

    % initial conditions
    l0 = ldesFunc(0,traj,freq);
    qB0 = qBdesFunc(0,traj,freq);

    M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
    tmp = inv(M)*[vx0_hip;vy0_hip];

    dl0 = tmp(1); dqB0 = tmp(2);

    x0 = 0;
    dx0 = 0; % initial foot position and velocity


    tspan = [0 stance_duration];

    q0 = [l0 dl0 qB0 dqB0 x0 dx0];

    % Set event function
    options = odeset('Events', @swim_event_func);

    [t, q] = ode45(@(t, q) odefun_unifiedstance(t,q, traj, freq, freeParams), tspan, q0, options);

    footPos = q(:,5);
    time = t;


end