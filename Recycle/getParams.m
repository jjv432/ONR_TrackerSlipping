function params = getParams()
params.g = 9.8;

params.mH = 18.8; %  % Hip mass (two legs in stance)
params.mQ = 23.8/1000; % Foot mass
params.k =  8003; %13592;     % leg stifness (N/m)


params.rho = 1000;
params.R = 0.1568;% 0.1970; % Radius of spherical hip used to match buoyancy


params.finLen = 10/100;
params.legWidth = 2/10; % leg width

params.epsilonV = 1E-5;
params.Cd_hip = 0.4484;
params.Cd_leg = 2.8118; % drag coefficient leg segment with fin


% Parameters that could vary ( We will add these as free params)
%params.finWidth = 5.5/100; %m
%params.muk = 0.7;
%params.mus = 1.35*params.muk;
%params.kp_ang = 40; 
%params.kd_ang = params.kp_ang/10;











