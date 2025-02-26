clc; clear; close all; format compact

addpath("src");
load("T_Results.mat");


ufm = UnifiedFrictionModel(t(15));
ufm.UnifiedModelLight([.5 .5 .5])
ufm.RunPSO;

ufm.UnifiedModelLight(ufm.OptimizedValues.OptimizedState)
%%

figure()
hold on
% Plot my best match to the data
plot(ufm.ODEVariables.time, ufm.ODEVariables.footPos)
% Plot the raw data
plot(ufm.DataObject.t(1:ufm.DataObject.StatsPlottingTrialLength), ufm.DataObject.MeanXPosition);
