clc; clear all
% Initial values
rotSpeedInitial = 1.2; % Rotor speed [rad/s]
windSpeedInitial = 7.1869; % Wind speed [m/s]
dt = 0.05; % Timestep in [s]
gamma = 20.0;

% Initialize estimator struct()
estimator = wsEstimatorObj('nrel5mw',dt,gamma,rotSpeedInitial,windSpeedInitial);

% Test it out
windSpeed = [];
t = dt:dt:150;
for i = 1:length(t)
    bladePitch = 3*pi/180.;
    genTq = 25000; % Generator torue in N*m
    rotSpeed = 1.08;  % Rotor speed in rad/s
    estimator.update(genTq, rotSpeed, bladePitch)
    windSpeed(i) = estimator.windSpeed;
end
clf;plot(t,windSpeed)