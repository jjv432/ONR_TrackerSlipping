clc; clear; close all; format compact

addpath("src");
addpath("ModelingFunctions\");

%% Getting some data

load("T_Results.mat");

global cur_x_vals
cur_x_vals = t(15).MeanXPosition;

%% Do PSO

LB = [
    0 % finwidth
    0 % muK
    0 %kp
];
UB = [
    10
    3
    75
];

options = optimoptions('particleswarm', 'MaxIterations', 2);
nvars = 3;
[x, FVAL] = particleswarm(@ModelSimulationCost,nvars, LB, UB, options);
%%
[footPos, time] = UnifiedModelLight(x);

%%
figure
hold on
plot(time, footPos, 'r');
plot(linspace(time(1), time(end), numel(cur_x_vals)), cur_x_vals, 'k');
legend("Best Simulation", "Experimental Data");
