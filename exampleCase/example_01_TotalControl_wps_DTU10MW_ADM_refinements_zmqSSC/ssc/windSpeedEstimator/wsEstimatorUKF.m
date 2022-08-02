classdef wsEstimatorUKF < matlab.mixin.Copyable %handle
    % WSESTIMATOR This is the main class of the wsEstimator program
    % This class is based on the Augmented Unscented Kalman Filter for wind speed estimation developed by Silva, J.G.   
    properties
        time
        windSpeed
        rotSpeed
        P
        estimatorProperties
        turbineProperties
		res
		Cp
    end
    
    methods
        function obj = wsEstimatorUKF(turbineType,dt,gamma,rotSpeedInitial,windSpeedInitial)
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
			%alpha=1e-3; % Parameter that defines the spread of the sigma points 
            %beta=2; % Parameter that incorporates prior knowledge of the distribution
			%alpha=1.22;
            %beta=0.5;
            alpha=1.22;
            beta=.5;
            ki=0; % Secondary scaling parameter
			lambda=(alpha^2)*(L+ki)-L;
			c=L+lambda;
			Wm=[lambda/c 0.5/c+zeros(1,2*L)]; % Weighted vector
			Wc=Wm;
			Wc(1)=Wc(1)+(1-alpha^2+beta);
			c=sqrt(c);
			
            % UKF proccedure
			Pc=c*chol(obj.P)';
			xVe=[obj.rotSpeed; obj.windSpeed];
			Y = [xVe xVe];
			X = [xVe Y+Pc Y-Pc]; % 5 samples
			L1 = size(X,2); % Number of sigma points
			X1 = zeros(L,L1);
			
			% Calculate aerodynamic torque (5 times)
			for j=1:L1
            tipSpeedRatio(j) = X(1,j)*rotorRadius/X(2,j); % Estimated tip speed ratio [-]
            %It could use rotSpeedMeasured instead of X(j,1) <<<
			Cp(j) = obj.turbineProperties.cpFun(tipSpeedRatio(j),pitchMeasured); % Power coefficient [-]
			if isnan(Cp(j))
                disp([ 'Cp of sample ' num2str(j) ' is out of the region of operation: TSR=' ...
                    num2str(tipSpeedRatio(j)) ', Pitch=' num2str(pitchMeasured) ' deg.'])
                %disp('Assuming windSpeed to be equal to the past time instant.')
			end
			obj.Cp=[obj.Cp, Cp];
            end
				Y2=zeros(L,1);
				for j=1:L1
				aerodynamicTorque(j) = 0.5*fluidDensity*rotorArea*((X(2,j)^3)/X(1,j))*Cp(j); % Torque [Nm]
                aerodynamicTorque(j) = max(aerodynamicTorque(j), 0.0); % Saturate torque to non-negative numbers
				X1(:,j) = [ X(1,j) + (dt/inertTot)*(aerodynamicTorque(j)-genTorqueMeasured*gbRatio) ;
				 X(2,j) ];  %
				Y2=Y2+Wm(j)*X1(:,j);
				end
				X2=X1-Y2;
				P1=X2*diag(Wc)*X2'+Q;
				
				L2=size(X1,2);
				Z1=zeros(s,L2);
				Y3=zeros(s,1);
				for i=1:L2                   
				Z1(:,i)=[1 0]*X(:,i);       
				Y3=Y3+Wm(i)*Z1(:,i);       
				end
				Z2=Z1-Y3;
				P2=Z2*diag(Wc)*Z2'+R;
				
				% Linear correlation
				P12=X2*diag(Wc)*Z2';
				K=P12*inv(P2);
				obj.res=rotSpeedMeasured-Y3;
				xVe=Y2+K*(obj.res);
                obj.rotSpeed = xVe(1);
				obj.windSpeed = xVe(2);
				obj.P=P1-K*P2*K';
                
            % Update time
            obj.time = obj.time + obj.estimatorProperties.dt;
        end
        
    end
end

