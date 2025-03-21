% Camilo Ordonez - 2025

function [dq] = odefun_unifiedstance(t,q, traj, freq, freeParams)

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

Fnormal = Fn

dq1 = dl; dq2 = d2l;
dq3 = dqB; dq4 = d2qB;
dq5 = dx;  dq6 = d2x;

dq = [dq1; dq2; dq3; dq4; dq5; dq6];

