function [turbineProperties] = estimator_nrel5mw()
% Generate the NREL 5MW performance curve

% OLD (information from FAST v8):
% perfCurves = load('perfCurves5MW.mat');
% [X,Y] = ndgrid(perfCurves.lambda,perfCurves.theta*pi/180);

% NEW (information straight from SOWFA):
load('cpInterpolant.mat')

% Define turbine properties
turbineProperties = struct(...
    'gearboxRatio',97.0,... % Gearbox ratio [-]
    'inertiaTotal',4.0469564E+07,... % Total inertia
    'rotorRadius',63.2,... % Rotor radius [m]
    'cpFun',cpInterpolant,... % Cp interpolant for TSR and blade pitch
    'fluidDensity',1.225); % Fluid density
	'GBEfficiency',0.944); % Gear box efficiency
end