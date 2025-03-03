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
            obj.PSOInfo.LB= [0 0 0];
            obj.PSOInfo.UB= [10 10 10];
            obj.PSOInfo.options = optimoptions('particleswarm', 'MaxIterations', 2, 'MaxTime', 2, 'MinNeighborsFraction',1, 'SwarmSize', 20, 'FunctionTolerance', 1, 'MaxStallTime', 2);
            obj.PSOInfo.nvars = 3;
            obj.SimulationInfo.init_traj = readmatrix("walk_test_2.txt");
            obj.getParams;
            obj.prepareODEParams;
            % Set event function
            obj.ODEVariables.options = odeset('MinStep', 1e-7, 'AbsTol', 1e-4, 'RelTol', 1e-3, 'Events', @swim_event_func);
        end



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%          PSO Methods          %%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function RunPSO(obj)

            p = polyfit(obj.DataObject.t(1:obj.DataObject.StatsPlottingTrialLength), obj.DataObject.MeanXPosition, 9);


            [OptimizedState, FVAL] = particleswarm(@ModelSimulationCost, obj.PSOInfo.nvars, obj.PSOInfo.LB, obj.PSOInfo.UB, obj.PSOInfo.options);
            % [OptimizedState, FVAL] = particleswarm(@ModelSimulationCost, obj.PSOInfo.nvars, obj.PSOInfo.LB, obj.PSOInfo.UB);

            obj.OptimizedValues.OptimizedState = OptimizedState;
            obj.OptimizedValues.FVAL = FVAL;


            function [Cost] = ModelSimulationCost(freeParams)

                % Run the simulation to determine the range of x values
                obj.UnifiedModelLight(freeParams);

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

            [t, q] = ode45(@(t, q) obj.odefun_unifiedstance(t,q, freeParams), obj.ODEVariables.tspan, obj.ODEVariables.q0, obj.ODEVariables.options);
            % [t, q] = ode45(@(t, q) odefun_unifiedstance(t,q, obj.ODEVariables.traj, obj.SimulationInfo.freq, freeParams), obj.ODEVariables.tspan, obj.ODEVariables.q0);

            obj.ODEVariables.footPos = q(:,5);
            obj.ODEVariables.time = t;

        end


        function [dq] = odefun_unifiedstance(obj,t,q,freeParams)

            Tau = freeParams.kp_ang * (obj.qBdesFunc(t) - q(3)) +  freeParams.kd_ang * (obj.dqBdesFunc(t) - q(4));

            % % Apply switching logic
            % isSliding = 1;
            %
            %
            % switch(isSliding)
            %     case 0      % sticking
            %         % Force of friction is unknown and d2x = 0
            %         % Use stick dynamics
            %
            %         Maug = obj.M_aug_func_stick([q(1); q(3)]);
            %         fside = obj.f_func_stick([q(1); q(3); Tau; obj.ldesFunc(t)],[q(2); q(4)], obj.getFluidForcesUnified(q, freeParams));
            %
            %         ddq_aug_stick = Maug^-1*fside; % [d2l d2qB Ff Fn]
            %
            %         obj.ODEVariables.Fnormal = ddq_aug_stick(4); % normal
            %
            %         if(  abs(ddq_aug_stick(3)) > freeParams.mus*abs(obj.ODEVariables.Fnormal) )
            %             isSliding = 1;
            %         end
            %
            %         dq = [q(2); ddq_aug_stick(1); q(4); ddq_aug_stick(2); q(6); 0];
            %
            %
            %     case 1      %sliding
            %         % Force of friction is specified and d2x is unknown
            %         % Use continuous friction
            %         Maug = obj.M_aug_func_slide([q(1); q(3)],[q(6)],[freeParams.muk]);
            %         fside = obj.f_func_slide([q(1); q(3); Tau; obj.ldesFunc(t)],[q(2); q(4)],obj.getFluidForcesUnified(q, freeParams));
            %
            %         ddq_aug_slide = Maug^-1*fside; % ddq_aug = [ddl ddqB ddx Fn];
            %
            %         if ( ( abs(q(6)) < obj.SimulationInfo.params.epsilonV ) && ( abs(-freeParams.muk*q(6)*abs(ddq_aug_slide(4))/(abs(q(6)) + obj.SimulationInfo.params.epsilonV)) < freeParams.mus*abs(ddq_aug_slide(4)) ) )
            %             isSliding = 0;
            %         end
            %
            %         dq = [q(2); ddq_aug_slide(1); q(4); ddq_aug_slide(2); q(6); ddq_aug_slide(3)];
            %         obj.ODEVariables.Fnormal = ddq_aug_slide(4);
            %
            % end

            Maug = obj.M_aug_func_slide([q(1); q(3)],[q(6)],[freeParams.muk]);
            fside = obj.f_func_slide([q(1); q(3); Tau; obj.ldesFunc(t)],[q(2); q(4)],obj.getFluidForcesUnified(q, freeParams));

            ddq_aug_slide = Maug^-1*fside; % ddq_aug = [ddl ddqB ddx Fn];

            if ( ( abs(q(6)) < obj.SimulationInfo.params.epsilonV ) && ( abs(-freeParams.muk*q(6)*abs(ddq_aug_slide(4))/(abs(q(6)) + obj.SimulationInfo.params.epsilonV)) < freeParams.mus*abs(ddq_aug_slide(4)) ) )
                isSliding = 0;
            end

            dq = [q(2); ddq_aug_slide(1); q(4); ddq_aug_slide(2); q(6); ddq_aug_slide(3)];
            obj.ODEVariables.Fnormal = ddq_aug_slide(4);

        end

        function Maug = M_aug_func_slide(obj,in1,dx,muk)
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
        function Maug = M_aug_func_stick(obj,in1)
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
        function f = f_func_stick(obj,in1,in2,in3)
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

        function f = f_func_slide(obj,in1,in2,in3)
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



        function qBdes = qBdesFunc(obj,t)
            idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
            if(idx==0)
                idx = 1;
            end
            xdes = obj.ODEVariables.traj(idx,1);
            zdes = obj.ODEVariables.traj(idx,2);
            qBdes = atan2(zdes,xdes) + pi;
        end

        function ldes = ldesFunc(obj,t)
            idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
            if(idx==0)
                idx = 1;
            end
            xdes = obj.ODEVariables.traj(idx,1);
            zdes = obj.ODEVariables.traj(idx,2);
            ldes = sqrt(xdes^2+zdes^2);
        end

        function dqBdes = dqBdesFunc(obj,t)
            idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
            if(idx==0)
                idx = 1;
            end
            xdes = obj.ODEVariables.traj(idx,1);
            zdes = obj.ODEVariables.traj(idx,2);
            qBdestemp = atan2(zdes,xdes);

            xdes1 = obj.ODEVariables.traj(idx+1,1);
            zdes1 = obj.ODEVariables.traj(idx+1,2);
            qBdes1 = atan2(zdes1,xdes1);

            dt = 1/obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));
            dqBdes = (qBdes1-qBdestemp)/dt;
        end

        function dldes = dldesFunc(obj,t)
            idx = ceil(t*length(obj.ODEVariables.traj(:,1))*obj.SimulationInfo.freq);
            if(idx==0)
                idx = 1;
            end
            xdes = obj.ODEVariables.traj(idx,1);
            zdes = obj.ODEVariables.traj(idx,2);
            ldes_temp = sqrt(xdes^2+zdes^2);

            xdes1 = obj.ODEVariables.traj(idx+1,1);
            zdes1 = obj.ODEVariables.traj(idx+1,2);
            ldes1 = sqrt(xdes1^2+zdes1^2);

            dt = 1/obj.SimulationInfo.freq/length(obj.ODEVariables.traj(:,1));

            dldes = (ldes1-ldes_temp)/dt;

        end

        function [Fluid_forces] = getFluidForcesUnified(obj, q,freeParams)

            % Drag on leg - Integration
            Fdrag = [0;0;0]; % drag force on leg
            Torque_water = [0;0;0]; % torque of drag force about foot;

            % iter = 1;
            for sst = obj.SimulationInfo.ss_temp % integrate over fin

                vel_p = [q(6);0;0] + q(4)*sst*[-sin(q(3)); cos(q(3));0]; % velocity of point p. Located at distance ss from the foot.
                vel_hat = vel_p/norm(vel_p + obj.SimulationInfo.params.epsilonV); % unit vector along velocity at point p

                lambda_hat = cross(vel_hat,[0;0;1]); % unit vector perpendicular to velocity of p

                segment_len_projection = abs( dot(obj.SimulationInfo.delta_s*[cos(q(3));sin(q(3));0], lambda_hat) ) ; % length of projection of ds onto lamba

                Frontal_area = segment_len_projection*freeParams.finWidth;

                Fdrag_segment = -0.5*obj.SimulationInfo.params.rho*obj.SimulationInfo.params.Cd_leg*Frontal_area*vel_p*norm(vel_p); % segment drag contribution.
                TorqueDrag_segment = cross(sst*[cos(q(3));sin(q(3));0],Fdrag_segment); % about foot

                Fdrag = Fdrag + Fdrag_segment;
                Torque_water = Torque_water + TorqueDrag_segment;

                % obj.SimulationInfo.deltaDrags(iter,:) = [Fdrag_segment(1) Fdrag_segment(2) sst];
                % iter = iter + 1;

            end

            Torque_water_Bcm = Torque_water - cross(0.5*obj.SimulationInfo.params.finLen*[cos(q(3));sin(q(3));0],Fdrag); % torque of water about cm of fin.

            vel_H = q(2)*[cos(q(3));sin(q(3));0] + q(1)*q(4)*[-sin(q(3)); cos(q(3));0] + q(6)*[1;0;0]; % velocity of the hip

            Fdrag_hip = -0.5*obj.SimulationInfo.params.rho*pi*obj.SimulationInfo.params.Cd_hip*obj.SimulationInfo.params.R^2*vel_H*norm(vel_H);

            Fluid_forces = [Fdrag_hip(1); Fdrag_hip(2); Fdrag(1); Fdrag(2); obj.SimulationInfo.params.rho*obj.SimulationInfo.Vol_hip*obj.SimulationInfo.params.g; Torque_water_Bcm(3)];


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

            % getFluidForces params that don't change
            % obj.SimulationInfo.delta_s = 0.01;
            obj.SimulationInfo.delta_s = .05; % making much larger so sim doesnt take forever
            obj.SimulationInfo.Vol_hip = (4/3)*pi*obj.SimulationInfo.params.R^3;
            obj.SimulationInfo.ss_temp = 0:obj.SimulationInfo.delta_s:obj.SimulationInfo.params.finLen;
            obj.SimulationInfo.numForces = numel(obj.SimulationInfo.ss_temp);
            obj.SimulationInfo.deltaDrags = zeros(obj.SimulationInfo.numForces,3);
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
            l0 = obj.ldesFunc(0);
            qB0 = obj.qBdesFunc(0);

            M = [cos(qB0) -l0*sin(qB0); sin(qB0) l0*cos(qB0)];
            tmp = M\[obj.SimulationInfo.vx0_hip; obj.SimulationInfo.vy0_hip];

            dl0 = tmp(1); dqB0 = tmp(2);

            x0 = 0;
            dx0 = 0; % initial foot position and velocity

            %% PUT THIS BACK! (remove the fraction)
            obj.ODEVariables.tspan = [0, obj.SimulationInfo.stance_duration/4];

            obj.ODEVariables.q0 = [l0 dl0 qB0 dqB0 x0 dx0];


        end


    end
end