function [turbineProperties] = estimator_dtu10mw()
% NEW (information straight from SOWFA):
load('cpInterp_DTU10MW_SOWFA.mat')
load('ctInterp_DTU10MW_SOWFA.mat')

% Define turbine properties
turbineProperties = struct(...
    'gearboxRatio',50.0,... % Gearbox ratio [-]
    'inertiaTotal',1.409969209E+08,... % Total inertia
    'rotorRadius',89.2,... % Rotor radius [m]
	'ctFun',ctInterpolant,...
    'cpFun',cpInterpolant,... % Cp interpolant for TSR and blade pitch
	'GBEfficiency',1,...
    'fluidDensity',1.23); % Fluid density
end