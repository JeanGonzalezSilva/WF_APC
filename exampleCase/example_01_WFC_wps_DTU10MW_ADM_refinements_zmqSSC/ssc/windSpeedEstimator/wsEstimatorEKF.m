classdef wsEstimatorEKF < matlab.mixin.Copyable %handle
    % WSESTIMATOR This is the main class of the wsEstimator program
    % This class is based on the Augmented Unscented Kalman Filter for wind speed estimation developed by Silva, J.G.   
    properties
        time
        windSpeed
        rotSpeed
        P
        estimatorProperties
        turbineProperties
		Cp
		dCp
		res
    end
    
    methods
        function obj = wsEstimatorEKF(turbineType,dt,gamma,rotSpeedInitial,windSpeedInitial)
            %wsEstimator Construct an instance of this class
            if strcmp(lower(turbineType),'nrel5mw')
                turbineProperties = estimator_nrel5mw();
            elseif strcmp(lower(turbineType),'dtu10mw')
                turbineProperties = estimator_dtu10mw();
            else
                error('Cannot find properties file for this turbine.')
            end
            if nargin < 3
                gamma = 0.1; % Adaptation gain of the estimator, should be non-negative
            end
            if nargin < 4
                rotSpeedInitial = 1.0; % Rotor speed [rad/s]
                windSpeedInitial = 7.5; % Wind speed [m/s]
            end
            % Save to self
            obj.estimatorProperties = struct('dt',dt,'gamma',gamma);
            obj.turbineProperties = turbineProperties;          
            obj.setInitialValues(rotSpeedInitial,windSpeedInitial);
        end
         
        function setInitialValues(obj, rotSpeedInitial, windSpeedInitial)
            % Update wind speed and time
            obj.time = 0;
			obj.rotSpeed = rotSpeedInitial;
            obj.windSpeed = windSpeedInitial;
            gamma = obj.estimatorProperties.gamma;
			% Filter initial conditions
            pi1=1;
            pi2=1;
			obj.P=[pi1 0; 0 pi2]; % Initial state covariance
			qi1=.0001;  
			ri1=.01; 
			obj.estimatorProperties.Q=[qi1 0 ;0 gamma]; % Process noise covariance
			obj.estimatorProperties.R=ri1; % Measurement noise covariance
			obj.Cp = [];
            obj.dCp = 0;
        end
        
        function update(obj, genTorqueMeasured, rotSpeedMeasured, pitchMeasured)         
            % Variables from obj
            fluidDensity = obj.turbineProperties.fluidDensity; % Fluid density [kg/m3]
            rotorRadius = obj.turbineProperties.rotorRadius; % Rotor radius [m]
            rotorArea = pi*rotorRadius^2; % Rotor swept surface area [m2]
            gbRatio = obj.turbineProperties.gearboxRatio; % Gearbox ratio
            inertTot = obj.turbineProperties.inertiaTotal; % Inertia
            dt = obj.estimatorProperties.dt; % Timestep
           
            % UKF parameters
			Q=obj.estimatorProperties.Q;
			R=obj.estimatorProperties.R;
			L=2;  % Number of states
			s=1; % Number of measurements
			P=obj.P;
			
			xVe=[obj.rotSpeed; obj.windSpeed];
			
			% Calculate the estimated Cp
			tipSpeedRatio = xVe(1)*rotorRadius/xVe(2); % Estimated tip speed ratio [-]
            Cp = obj.turbineProperties.cpFun(tipSpeedRatio,pitchMeasured); % Power coefficient [-]
			if isnan(Cp)
                disp([ 'Cp is out of the region of operation: TSR=' ...
                    num2str(tipSpeedRatio) ', Pitch=' num2str(pitchMeasured) ' deg.'])
                %disp('Assuming windSpeed to be equal to the past time instant.')
			end
			obj.Cp=[obj.Cp, Cp];
		
			% Jacobian Matrices avaliated at the current point
			A11=[1 - (dt/inertTot)*0.5*fluidDensity*rotorArea*((xVe(2)^3)/(xVe(1)^2))*Cp + (dt/inertTot)*0.5*fluidDensity*rotorArea*((xVe(2)^2)/xVe(1))*rotorRadius*obj.dCp];
			A12=[3*(dt/inertTot)*0.5*fluidDensity*rotorArea*((xVe(2)^2)/xVe(1))*Cp- (dt/inertTot)*0.5*fluidDensity*rotorArea*xVe(2)*rotorRadius*obj.dCp];
			A21= 0;
			A22= 1;
			A=[A11 A12; A21 A22];
			%B=[-dt/inertTot; 0];
			C=[1 0];
			
            % EKF proccedure
			% Prediction
			aerodynamicTorque = 0.5*fluidDensity*rotorArea*((xVe(2)^3)/xVe(1))*Cp; % Torque [Nm]
            aerodynamicTorque = max(aerodynamicTorque, 0.0); % Saturate torque to non-negative numbers
			xVe(1)=xVe(1) + (dt/inertTot)*(aerodynamicTorque-genTorqueMeasured*gbRatio);
			P=A*P*A' + Q;
			yVe=xVe(1);
            
			%Correction
			obj.res = rotSpeedMeasured-yVe;
            Pzz=C*P*C'+R;
			K=P*C'*inv(Pzz);
			xVe=xVe+K*(obj.res);
            obj.rotSpeed = xVe(1);
            obj.windSpeed = xVe(2);
            
			%obj.P=(eye(L) - K*C)*P;
            %obj.P= P - K*C*P;
            %obj.P= P - K*(Pzz)*K';
			obj.P= (eye(L)-K*C)*P*(eye(L)-K*C)' + K*R*K'; %Joseph Form
			
			
            % Update time
            obj.time = obj.time + obj.estimatorProperties.dt;
			
			% Save Cp for derivative calculation
            int=1;
			if obj.time<(int+dt)
			dCp=0;
			else
			dCp= Cp(1) - obj.Cp(round((obj.time-int)/dt)); % Derivative considering the interval of 1 second
			end
			obj.dCp=dCp; 
        end
        
    end
end

