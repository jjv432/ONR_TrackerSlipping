classdef UnifiedFrictionModel < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        params
        DataObject
        IndexInfo
        PSOInfo
        OptimizedValues
        SimulationInfo
        ODEVariables
        dataSpline


    end

    methods

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%          CONSTRUCTOR          %%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = UnifiedFrictionModel(DataObject)
            obj.DataObject = DataObject;
            obj.PSOInfo.LB= [5 0 60];
            obj.PSOInfo.UB= [10 3 75];
            obj.PSOInfo.options = optimoptions('particleswarm', 'MaxIterations', 100);
            obj.PSOInfo.nvars = 3;
            obj.SimulationInfo.init_traj = readmatrix("walk_test_2.txt");
        end



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%          PSO Methods          %%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function RunPSO(obj)
            obj.getParams;
            obj.prepareODEParams;


            [OptimizedState, FVAL] = particleswarm(@ModelSimulationCost, obj.PSOInfo.nvars, obj.PSOInfo.LB, obj.PSOInfo.UB, obj.PSOInfo.options);

            obj.OptimizedValues.OptimizedState = OptimizedState;
            obj.OptimizedValues.FVAL = FVAL;
            function [Cost] = ModelSimulationCost(freeParams)

                % Run the simulation to determine the range of x values
                obj.UnifiedModelLight(freeParams);

                p = polyfit(obj.DataObject.t(1:obj.DataObject.StatsPlottingTrialLength), obj.DataObject.MeanXPosition, 9);
                obj.dataSpline = polyval(p, obj.ODEVariables.time);

                Cost = sum(abs((obj.ODEVariables.footPos - obj.dataSpline)));
            end
        end



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%       ODE45 Functions         %%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function UnifiedModelLight(obj, newFreeParams)


            % Free Params used for Model Tunning
            freeParams.finWidth = newFreeParams(1);
            freeParams.muk = newFreeParams(2);
            freeParams.kp_ang = newFreeParams(3);
            freeParams.kd_ang = (1/10)*freeParams.kp_ang;
            freeParams.mus = 1.3*freeParams.muk;

            % Set event function
            % obj.ODEVariables.options = odeset('Events', @swim_event_func);

            % [t, q] = ode45(@(t, q) odefun_unifiedstance(t,q, obj.ODEVariables.traj, obj.SimulationInfo.freq, freeParams), obj.ODEVariables.tspan, obj.ODEVariables.q0, obj.ODEVariables.options);
            [t, q] = ode45(@(t, q) odefun_unifiedstance(t,q, obj.ODEVariables.traj, obj.SimulationInfo.freq, freeParams), obj.ODEVariables.tspan, obj.ODEVariables.q0);

            obj.ODEVariables.footPos = q(:,5);
            obj.ODEVariables.time = t;

            function [dq] = odefun_unifiedstance(t,q, traj, freq, freeParams)

                l = q(1); dl = q(2);
                qB = q(3); dqB = q(4);
                x = q(5); dx = q(6);

                kp_ang = freeParams.kp_ang; % controller gains for applied torque on leg
                kd_ang = freeParams.kd_ang;
                muk = freeParams.muk;
                mus = freeParams.mus;


                ldes = ldesFunc(t,traj,freq); % desired leg length
                qBdes = qBdesFunc(t,traj,freq); % desired leg angle from horizontal

                dldes = dldesFunc(t,traj,freq); % desired rates
                dqBdes = dqBdesFunc(t,traj,freq);

                Tau = kp_ang * (qBdes - qB) +  kd_ang * (dqBdes - dqB);

                % Compute Fluid Forces (drag forces on hip h, drag forces on leg l, torque
                % due to water
                [Fluid_forces, deltaLegDrags] = getFluidForcesUnified(q, freeParams);

                % Apply switching logic
                isSliding = 0;


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

                obj.ODEVariables.Fnormal = Fn;

                dq1 = dl; dq2 = d2l;
                dq3 = dqB; dq4 = d2qB;
                dq5 = dx;  dq6 = d2x;

                dq = [dq1; dq2; dq3; dq4; dq5; dq6];


                function qBdes = qBdesFunc(t,traj,freq)
                    idx = ceil(t*length(traj(:,1))*freq);
                    if(idx==0)
                        idx = 1;
                    end
                    xdes = traj(idx,1);
                    zdes = traj(idx,2);
                    qBdes = atan2(zdes,xdes) + pi;
                end

                function ldes = ldesFunc(t,traj,freq)
                    idx = ceil(t*length(traj(:,1))*freq);
                    if(idx==0)
                        idx = 1;
                    end
                    xdes = traj(idx,1);
                    zdes = traj(idx,2);
                    ldes = sqrt(xdes^2+zdes^2);
                end

            end


            % function [value,isterminal,direction] = swim_event_func(obj)
            %     value = obj.ODEVariables.Fnormal;
            %     isterminal = 1;  % Stop integration when event occurs
            %     direction = -1;  % Detect when Fn crosses zero from positive to negative
            % end
        end














        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%       Param Organization      %%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function getParams(obj)
            obj.SimulationInfo.params.g = 9.8;

            obj.SimulationInfo.params.mH = 18.8; %  % Hip mass (two legs in stance)
            obj.SimulationInfo.params.mQ = 23.8/1000; % Foot mass
            obj.SimulationInfo.params.k =  8003; %13592;     % leg stifness (N/m)


            obj.SimulationInfo.params.rho = 1000;
            obj.SimulationInfo.params.R = 0.1568;% 0.1970; % Radius of spherical hip used to match buoyancy


            obj.SimulationInfo.params.finLen = 10/100;
            obj.SimulationInfo.params.legWidth = 2/10; % leg width

            obj.SimulationInfo.params.epsilonV = 1E-5;
            obj.SimulationInfo.params.Cd_hip = 0.4484;
            obj.SimulationInfo.params.Cd_leg = 2.8118; % drag coefficient leg segment with fin

            obj.SimulationInfo.freq = 2.5; %Hz
            obj.SimulationInfo.stance_duration = 0.16;
            obj.SimulationInfo.vx0_hip = 0.2;
            obj.SimulationInfo.vy0_hip = -0.0; % initial velocity of hip at stance m/s.
            % Should vary with frequency
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%        ODE45 Params           %%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        function prepareODEParams(obj)

            %a Trajectory sent to the robot
            x = obj.SimulationInfo.init_traj(1:500,5);
            z = obj.SimulationInfo.init_traj(1:500,7);

            obj.ODEVariables.traj = [x,z];

            % initial conditions
            l0 = ldesFunc(0,obj.ODEVariables.traj,obj.SimulationInfo.freq);
            qB0 = qBdesFunc(0,obj.ODEVariables.traj,obj.SimulationInfo.freq);

            M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
            tmp = M\[obj.SimulationInfo.vx0_hip; obj.SimulationInfo.vy0_hip];

            dl0 = tmp(1); dqB0 = tmp(2);

            x0 = 0;
            dx0 = 0; % initial foot position and velocity


            obj.ODEVariables.tspan = [0 obj.SimulationInfo.stance_duration];

            obj.ODEVariables.q0 = [l0 dl0 qB0 dqB0 x0 dx0];


        end


    end
end