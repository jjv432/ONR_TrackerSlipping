clc; clear; close all; format compact

addpath("src");
addpath("ReducedOrderModel");

%% Getting some data

load("T_Results.mat");

cur_x_vals = t(15).MeanXPosition;

%% Do PSO

nvars = 3;
[x, FVAL] = particleswarm(@ModelSimulationCost,nvars);