% Camilo Ordonez - 2025

clc
clear
close all

addpath("autoSlide\")
addpath("autoStick\")


traj = readmatrix("walk_test_2.txt");


params = getParams();

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




