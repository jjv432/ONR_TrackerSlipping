
function [Fluid_forces, deltaLegDrags] = getFluidForcesUnified(q,freeParams)
l = q(1); dl = q(2);  % leg length and rate
qB = q(3); dqB = q(4); % leg angle and rate from ground
x = q(5); dx = q(6);   % foot position and vel

params = getParams();
g = params.g;
epsilonV = params.epsilonV;
Cd_leg = params.Cd_leg; % drag coefficient leg segment
Cd_hip = params.Cd_hip;
LB = params.finLen; % length of leg with fin
hB = params.legWidth; % leg width
rho = params.rho; % densitiy of water
R = params.R;         % Radius of hip


wB = freeParams.finWidth; %m

Vol_hip = (4/3)*pi*R^3;

Fbuoy_hip_y = rho*Vol_hip*g;

% Drag on leg - Integration
delta_s = 0.01;
bx_hat = cos(qB)*[1;0;0] + sin(qB)*[0;1;0];
by_hat = -sin(qB)*[1;0;0] + cos(qB)*[0;1;0];

Fdrag = [0;0;0]; % drag force on leg
Torque_water = [0;0;0]; % torque of drag force about foot;
                        
ss_tmp = 0:delta_s:LB; %iterate over fin
numForces = numel(ss_tmp);
deltaLegDrags = zeros(numForces,3);
iter = 1;
for ss = 0:delta_s:LB % integrate over fin
    
    %vel_p = [dx;0;0] + dqB*ss*by_hat;
    vel_p = [dx;0;0] + dqB*ss*by_hat; % velocity of point p. Located at distance ss from the foot.
    vel_hat = vel_p/norm(vel_p + epsilonV); % unit vector along velocity at point p

    lambda_hat = cross(vel_hat,[0;0;1]); % unit vector perpendicular to velocity of p
    
    segment_len_projection = abs( dot(delta_s*bx_hat, lambda_hat) ) ; % length of projection of ds onto lamba

    Frontal_area = segment_len_projection*wB;

    Fdrag_segment = -0.5*rho*Cd_leg*Frontal_area*vel_p*norm(vel_p); % segment drag contribution.
    TorqueDrag_segment = cross(ss*bx_hat,Fdrag_segment); % about foot
    
    Fdrag = Fdrag + Fdrag_segment;
    Torque_water = Torque_water + TorqueDrag_segment;     
   
    deltaLegDrags(iter,:) = [Fdrag_segment(1) Fdrag_segment(2) ss];
    iter = iter + 1;
end
    
Torque_water_Bcm = Torque_water - cross(0.5*LB*bx_hat,Fdrag); % torque of water about cm of fin.
Fdrag_leg_x = Fdrag(1);
Fdrag_leg_y = Fdrag(2);
%keyboard()
Torque_water_Bcm_measure = Torque_water_Bcm(3);

vel_H = dl*bx_hat + l*dqB*by_hat + dx*[1;0;0]; % velocity of the hip

Fdrag_hip = -0.5*rho*pi*Cd_hip*R^2*vel_H*norm(vel_H);

Fdrag_hip_x = Fdrag_hip(1);
Fdrag_hip_y = Fdrag_hip(2);

Fluid_forces = [Fdrag_hip_x; Fdrag_hip_y; Fdrag_leg_x; Fdrag_leg_y; Fbuoy_hip_y; Torque_water_Bcm_measure];

%Fluid_forces = [0;0;0;0;Fbuoy_hip_y;0];













end