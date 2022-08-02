classdef wsEstimatorObj < matlab.mixin.Copyable %handle
    %WSESTIMATOR This is the main class of the wsEstimator program
    %   This class is based on the I&I Wind Speed estimator developed by
    %   Ortega et al.
    
    properties
        time
        windSpeed
        estimatorProperties
        turbineProperties
    end
    
    methods
        function obj = wsEstimatorObj(turbineType,dt,gamma,rotSpeedInitial,windSpeedInitial)
            %wsEstimator Construct an instance of this class
            if strcmp(lower(turbineType),'nrel5mw')
                turbineProperties = estimator_nrel5mw();
			elseif strcmp(lower(turbineType),'dtu10mw')
                turbineProperties = estimator_dtu10mw();
            else
                error('Cannot find properties file for this turbine.')
            end
            
            if nargin < 3
                gamma = 20; % Adaptation gain of the estimator, should be non-negative
            end
            if nargin < 4
                rotSpeedInitial = 1.0; % Rotor speed [rad/s]
                windSpeedInitial = 7.5; % Wind speed [m/s]
            end

            % Save to self
            obj.estimatorProperties = struct('dt',dt,'gamma',gamma);
            obj.turbineProperties = turbineProperties;
            
            % Set initial estimates (wind speed, integrator)
            obj.setInitialValues(rotSpeedInitial,windSpeedInitial);
        end
        
        
        function setInitialValues(obj, rotSpeedInitial, windSpeedInitial)
            % Update wind speed and time
            obj.time = 0;
            obj.windSpeed = windSpeedInitial;
            
            % Update integrator state
            gamma = obj.estimatorProperties.gamma;
            integratorInitial = windSpeedInitial-gamma*rotSpeedInitial;
            obj.estimatorProperties.integratorState = integratorInitial;
        end
        
        
        function update(obj, genTorqueMeasured, rotSpeedMeasured, pitchMeasured)
            % Import variables from obj
            fluidDensity = obj.turbineProperties.fluidDensity; % Fluid density [kg/m3]
            rotorRadius = obj.turbineProperties.rotorRadius; % Rotor radius [m]
            rotorArea = pi*rotorRadius^2; % Rotor swept surface area [m2]
            gamma = obj.estimatorProperties.gamma; % Estimator gain
            gbRatio = obj.turbineProperties.gearboxRatio; % Gearbox ratio
            inertTot = obj.turbineProperties.inertiaTotal; % Inertia
            dt = obj.estimatorProperties.dt; % Timestep
            
            % Calculate aerodynamic torque
            tipSpeedRatio = rotSpeedMeasured*rotorRadius/obj.windSpeed; % Estimated tip speed ratio [-]
            Cp = obj.turbineProperties.cpFun(tipSpeedRatio,pitchMeasured); % Power coefficient [-]
            
            if isnan(Cp)
                disp(['Cp is out of the region of operation: TSR=' ...
                    num2str(tipSpeedRatio) ', Pitch=' num2str(pitchMeasured) ' deg.'])
                disp('Assuming windSpeed to be equal to the past time instant.')
            else
                aerodynamicTorque = 0.5*fluidDensity*rotorArea*((obj.windSpeed^3)/rotSpeedMeasured)*Cp; % Torque [Nm]
                aerodynamicTorque = max(aerodynamicTorque, 0.0); % Saturate torque to non-negative numbers
                
                % Update estimator state and wind speed estimate
                VwIdot = gamma / inertTot*(genTorqueMeasured*gbRatio - aerodynamicTorque);
                obj.estimatorProperties.integratorState = obj.estimatorProperties.integratorState + VwIdot*dt;
                obj.windSpeed = obj.estimatorProperties.integratorState + gamma*rotSpeedMeasured;
            end
            
            % Update time
            obj.time = obj.time + obj.estimatorProperties.dt;
        end
        
    end
end

