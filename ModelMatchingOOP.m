clc; clear; close all; format compact

addpath("src");
addpath("ModelingFunctions\");

load("T_Results.mat");


ufm = UnifiedFrictionModel(t(15));
ufm.RunPSO;

ufm.UnifiedModelLight(ufm.OptimizedValues.OptimizedState)
%%

figure()
hold on
plot(ufm.ODEVariables.time, ufm.ODEVariables.footPos)
plot(ufm.DataObject.t(1:ufm.DataObject.StatsPlottingTrialLength), ufm.DataObject.MeanXPosition);
