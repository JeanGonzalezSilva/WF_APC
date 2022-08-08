%% SSC EXAMPLE FUNCTION
addpath(genpath('windSpeedEstimator'))

% Setup zeroMQ server
zmqServer = zeromqObj('/home/jgonzalessilva/OpenFOAM/jgonzalessilva-2.4.0/jeromq/jeromq-0.4.4-SNAPSHOT.jar',1771,3600,true);

% Load the yaw setpoint LUT and set-up a simple function
nTurbs = 32;

% Initial control settings
degRad=pi()/180.0;
bladePitchFine = 0.013090000000/degRad; %From ROSCO [deg]
torqueArrayOut     = 0.0 *ones(1,nTurbs); % Not used unless 'torqueSC' set in turbineProperties
yawAngleArrayOut   = 120.*ones(1,nTurbs); % Not used unless 'yawSC' set in turbineProperties
pitchAngleArrayOut = bladePitchFine *ones(1,nTurbs); % Not used unless 'PIDSC' or 'pitchSC' set in turbineProperties

% Reference signal
load('AGCdata')
initialtime=20000;
dt = 0.50;
timeSetpointArray = [initialtime+dt:dt:initialtime+1000];
powerSetpointFarmArray=zeros(1,size(timeSetpointArray,2));
for ii=1:(size(timeSetpointArray,2))
if timeSetpointArray(ii)>20300
    powerSetpointFarmArray(ii) = interp1(AGCdata(:,1),AGCdata(:,2),(timeSetpointArray(ii)-initialtime+114),'linear');
end
end
powerSetpointFarmArray=powerSetpointFarmArray.*(nTurbs*0.5e6)+(nTurbs*3.5e6);

% Setup empty variable
estimatedThrustArray = [];
estimatedWindSpeedArray = [];
powerSetpointTurbArray = [];
powerAvailableTurbArray = [];
powerSetpointTurbArrayLC=[];
powerSetpointTurbArrayNC = [];
powerSetpointTurbArrayNTC = [];
up = [];
ut = [];
sumNonSatutArray = [];
rotorSpeedReferenceArray=[];
indtrackingThrustVarIntArray=[];
varThrustArray=[];

% Initialization of the effective wind speed estimator 
gamma = 40.0;
beta= 10.0;
windSpeedInitial=11*ones(nTurbs,1);
rotSpeedInitial = 1.0*ones(nTurbs,1);
% Loop initialization
firstRun = true;

% Setup wind farm power controller
applyFarmPowerControl = true;
trackingErrorInt = 0;
wfcPidGainKP = 0;
wfcPidGainKI = 1 / (2*nTurbs*dt);
powerSetpointVariationPowerControl=zeros(1,nTurbs);
turbIsGreedy = zeros(1,nTurbs);

% Setup wind farm thrust controller
applyFarmThrustControl = true;
trackingThrustVarInt = zeros(1,nTurbs);
wfcPidThrustGainKI = 0.5;
wfcPidThrustGainKP=0;
powerSetpointVariationThrustControl=zeros(1,nTurbs);
sumNonSatut=0;
varThrust=zeros(1,nTurbs);

%Individual Thrust Tracking
indtrackingThrustVarInt=0;
indThrustReference=500000; %N
indPidThrustGainKI=1.444;
powerSetpointVariationIndThrustControl=zeros(1,nTurbs);
indPidThrustGainKP=0;
indTurbine=18; %Turbine number
thrustScale=0.944;


%DTU 10MW parameters
powerRated = 10e6; %[W]
genEfficiency = 1; % Generator efficiency for the NREL 5MW turbine
Rr=89.2;
gbRatio=50;
fluidDensity=1.23;
RpmtoRad=9.5493;


%Pitch control parameters
InitializePitchControl1(1:nTurbs)=true;
InitializePitchControl2(1:nTurbs)=true;
rpmRadSec=2*pi()/60;
PitchMin = bladePitchFine; %[deg]
PitchMax = 90; %[deg]
PitchSwitch = 0.01745329; %[rad]
PitchSwitch = PitchSwitch/degRad; %[deg]
load('controlTables/gainSchedulingPitch_DTU10MW_ROSCO.mat')
load('controlTables/rotorSpeedInterpolant_DTU10MW_ROSCO_constt.mat')

%Torque control [Based on Jonkman 2009 and ROSCO]
VS_RtPwr = powerRated/genEfficiency; %Rated mechanical power in Region 3
KGen = 79.43986000000; % From ROSCO [Nm/((rad/s)^2)]
VS_CtInSp = 200.0/RpmtoRad; % Transitional generator speed (HSS side) bet [rad/s]
%Region 1.1/2 is defined to span the range of generator speed between 200
%rpm and 50% above this value (or 300rpm)
VS_Rgn2Sp = 300.0/RpmtoRad; %Transitional generator speed (HSS side) bet [rad/s]
VS_Rgn3MP=0.01745329; % Minimum pitch angle at which the torque is tracking [rad]
VS_Slope15 = ( KGen*VS_Rgn2Sp*VS_Rgn2Sp )/( VS_Rgn2Sp - VS_CtInSp );
VS_TrGnSp = 405.0/RpmtoRad; %[rad/s]
VS_SlPc = 10.0; % Rated generator slip percentage in Region 2
VS_SySp = (VS_TrGnSp)/( 1.0 + 0.01*VS_SlPc ); %Synchronous speed of region 2 1/2 induction [rad/s]
Region2EndGenTorque=KGen*VS_TrGnSp*VS_TrGnSp;
VS_Slope25 = Region2EndGenTorque/( VS_TrGnSp  - VS_SySp );
Affine_GenTorque= Region2EndGenTorque-VS_Slope25*VS_TrGnSp; %from the VS_Slope25 the affine parameter with the VS_slope25
VS_RtGnSp = (-Affine_GenTorque+sqrt((Affine_GenTorque^2)+4*VS_Slope25*0.95*VS_RtPwr))/(2*VS_Slope25); %Rated generator speed (HSS side) [rad/s]
PC_RefSpd = (1/0.95)*VS_RtGnSp;
RatedGenTorque=VS_RtPwr/PC_RefSpd; %[Nm]
VS_MaxTq = 250000.0; %Maximum generator torque in Region 3 (HSS side) [Nm]
rotorSpeedRated= PC_RefSpd/gbRatio; %[rad/s]
downRegulationMode=2;

% Initialization of filter for the blade pitch measurements
betaf=0.961; %cutfrequency of 0.4rad/s
bladePitchFiltered(1:nTurbs)=bladePitchFine*ones(1,nTurbs);

% Start control loop
disp(['Entering wind farm controller loop...']);
while 1
    % Receive information from SOWFA
    dataReceived = zmqServer.receive();
	currentTime  = dataReceived(1,1);
    measurementVector = dataReceived(1,2:end); % [powerGenerator[1], torqueRotor[1], thrust[1], powerGenerator[2], torqueRotor[2], thrust[2]]
    
    % Measurements: [genPower,rotSpeedF,azimuth,rotThrust,rotTorque,genTorque,nacYaw,bladePitch]
    generatorPowerArray = measurementVector(1:8:end);
    rotorSpeedArray     = measurementVector(2:8:end);
    azimuthAngleArray   = measurementVector(3:8:end);
    rotorThrustArray    = measurementVector(4:8:end);
    rotorTorqueArray    = measurementVector(5:8:end);
    genTorqueArray      = measurementVector(6:8:end);
    nacelleYawArray     = measurementVector(7:8:end);
    bladePitchArray     = measurementVector(8:8:end); % Pitch angles of blade[0] of each turbine
    
	%% Wind Speed Estimator
    if firstRun
        % Initialize a wind speed estimator for each turbine
        for ii = 1:nTurbs
            %WSE{ii} = wsEstimatorObj('dtu10mw',dt,gamma,rotSpeedInitial(ii),windSpeedInitial(ii));
			WSE{ii} = wsEstimatorImprovedIandI('dtu10mw',dt,gamma,beta,rotSpeedInitial(ii),windSpeedInitial(ii));
        end
        firstRun = false;
    end
    
	% Update wind speed estimator for each turbine
    for ii = 1:nTurbs
        WSE{ii}.update(genTorqueArray(ii), rotorSpeedArray(ii), bladePitchArray(ii));
        disp(['WS of Turbine[' num2str(ii) '] = ' num2str(WSE{ii}.windSpeed) ' m/s.'])
        powerAvailableTurb(ii)= (0.5 * fluidDensity * pi * Rr^2)*(WSE{ii}.windSpeed^3); 
		tsRatio=rotorSpeedArray(ii)*Rr/WSE{ii}.windSpeed;
		estimatedThrust(ii)=(0.5 * fluidDensity * pi * Rr^2)*(WSE{ii}.windSpeed^2)*WSE{ii}.turbineProperties.ctFun(tsRatio,bladePitchArray(ii))*thrustScale;
    end
    powerAvailableTurb=min(powerAvailableTurb,powerRated);
  
	% Update current farm power setpoint and divide between turbines 
    powerSetpointFarmCurrent = interp1(timeSetpointArray,powerSetpointFarmArray,currentTime,'nearest');
	for ii=1:nTurbs
    powerSetpointTurbCurrent(ii) = powerSetpointFarmCurrent/nTurbs;
    end
    powerSetpointTurbNC=powerSetpointTurbCurrent;
	
	% Apply load limiting feedback
	ii=indTurbine;
	if applyIndThrustTrackingControl && currentTime>20100 && ((estimatedThrust(ii)>indThrustReference) || (indtrackingThrustVarInt<0))
				varThrust(ii)=indThrustReference-estimatedThrust(ii);
                indtrackingThrustVarInt = indtrackingThrustVarInt + dt * varThrust(ii);
                powerSetpointVariationIndThrustControl(ii)=indPidThrustGainKP*varThrust(ii)+indPidThrustGainKI*indtrackingThrustVarInt;            
				turbIsGreedy(ii)=true;
				disp(['Turbine is constrained.'])
	else
	varThrust(ii)=0;
	indtrackingThrustVarInt=0; % Reset
	powerSetpointVariationIndThrustControl(ii)=0;
    end
	powerSetpointTurbCurrent(ii) = powerSetpointTurbCurrent(ii) + powerSetpointVariationIndThrustControl(ii);
	powerSetpointTurbLC=powerSetpointTurbCurrent;
	
	 % Apply wind farm power controller
    if applyFarmPowerControl
        powerSetpointFarmPrevious = interp1(timeSetpointArray,powerSetpointFarmArray,currentTime-dt,'nearest'); 
		powerErrorPrevious = powerSetpointFarmPrevious - sum(generatorPowerArray);
		powerSetpointVariationPowerControl=0;
		if (currentTime > 20100)
            % Freeze integrator error if all turbines are saturated and setpoint not going down
            if ~all(turbIsGreedy) || (powerErrorPrevious < 0) % Allow increase if not all turbines greedy, also allow if power setpoint goes down
                trackingErrorInt = trackingErrorInt + dt * powerErrorPrevious;
            end
        % Ask non-greedy controllers to compensate for losses elsewhere
        powerSetpointVariationPowerControl = wfcPidGainKP*powerErrorPrevious + wfcPidGainKI * trackingErrorInt;
		end
        powerSetpointTurbCurrent = powerSetpointTurbCurrent + powerSetpointVariationPowerControl;
    end
    powerSetpointTurbNTC=powerSetpointTurbCurrent;
	
	% Apply wind farm thrust controller
    if applyFarmThrustControl
       if (currentTime > 20100)
			meanThrust=0;
			M=0; % Number of non-saturated turbines
			for ii=1:nTurbs
				if turbIsGreedy(ii)==0
					meanThrust=meanThrust+rotorThrustArray(ii);
					M=M+1;
				end
            end
			meanThrust=meanThrust/M; % Mean thrust of the non-saturated turbines
            sumNonSatut=0;
			for ii = 1:nTurbs  
                if turbIsGreedy(ii)==0
				varThrust(ii)= meanThrust-rotorThrustArray(ii);
				trackingThrustVarInt(ii) = trackingThrustVarInt(ii) + dt * varThrust(ii);
				powerSetpointVariationThrustControl(ii)=wfcPidThrustGainKP*varThrust(ii)+wfcPidThrustGainKI*trackingThrustVarInt(ii);
                sumNonSatut=sumNonSatut+powerSetpointVariationThrustControl(ii);
                else
				varThrust(ii)=0;
                trackingThrustVarInt(ii)=trackingThrustVarInt(ii); % Set integrator - keeping value
                powerSetpointVariationThrustControl(ii)=powerSetpointVariationThrustControl(ii);
                end
            end
         powerSetpointTurbCurrent = powerSetpointTurbCurrent + powerSetpointVariationThrustControl;
       end
    end
	
	% Update control values for each turbine
    for ii = 1:nTurbs  
	    %% LP Filter for the blade pitch measurements
		bladePitchFiltered(ii)= bladePitchFiltered(ii)*betaf + (1-betaf)*bladePitchArray(ii);
		
		rotorSpeedReference(ii) = rotorSpeedInterpolant_DTU10MW(powerSetpointTurbCurrent(ii));    
        if (powerSetpointTurbCurrent(ii)>VS_RtPwr) || (rotorSpeedArray(ii)<rotorSpeedReference(ii) && bladePitchArray(ii)<PitchSwitch)
            turbIsGreedy(ii) = true;
			disp(['Turbine is greedy.'])
			% TORQUE CONTROLLER
            genSpeed = rotorSpeedArray(ii) * gbRatio; % in rad/s
            torqueGreedy   = KGen * (genSpeed.^2); % Greedy control signal (Does Kgen is set fot genSpeed or rotSpeed?)
            %torqueTracking =  VS_RtPwr / (genSpeed*genEfficiency); % Tracking control signal  
            torqueTracking =  RatedGenTorque; % Constant torque [also need to change the transition
                if (genSpeed > VS_RtGnSp)
                    disp(['Current torque control mode: Region 3.']);
                    torqueArrayOut(ii) = torqueTracking;
                elseif genSpeed< (VS_CtInSp)
                    torqueArrayOut(ii)=0;
                    disp(['Current torque control mode: Region 1.']);
                elseif genSpeed< (VS_Rgn2Sp)   
                    torqueArrayOut(ii)= VS_Slope15*( genSpeed - VS_CtInSp);
                    disp(['Current torque control mode: Region 1.1/2.']);
                elseif genSpeed< (VS_TrGnSp)
                    disp(['Current torque control mode: Region 2.']);
                    torqueArrayOut(ii) = torqueGreedy; % Perfect tracking whenever possible, otherwise fall back on greedy
                else
                    disp(['Current torque control mode: Region 2.1/2.']);
                    torqueArrayOut(ii)= Region2EndGenTorque + VS_Slope25*( genSpeed - VS_TrGnSp);
                end
       
				% BLADE PITCH CONTROLLER - Gain-scheduling PID CONTROL
                if rotorSpeedArray(ii)<rotorSpeedRated &&  bladePitchArray(ii)<PitchSwitch
                    pitchAngleArrayOut(ii)=bladePitchFine;
					InitializePitchControl1(ii)=true;
                else
					if InitializePitchControl1(ii)
							PitchControlKI(ii) = -Ki(bladePitchArray(ii)*degRad)*gbRatio;
							intSpeedError(ii) = (bladePitchArray(ii)*degRad) / PitchControlKI(ii);
							speedErrorLast(ii)= 0;
						InitializePitchControl1(ii)=false;
					end
				%Compute the gains
				PitchControlKP(ii)= -Kp(bladePitchFiltered(ii)*degRad)*gbRatio;
				PitchControlKI(ii)= -Ki(bladePitchFiltered(ii)*degRad)*gbRatio;
				PitchControlKD(ii)= -Kd(bladePitchFiltered(ii)*degRad)*gbRatio;
				%Compute the low speed shaft speed error.
				speedError(ii)= rotorSpeedArray(ii)- rotorSpeedRated; %[rad/s]
				%Numerically integrate the speed error over time.
				intSpeedError(ii)= intSpeedError(ii)+speedError(ii)*dt;
				%Numerically take the deriviative of speed error w.r.t time.
				derivSpeedError(ii) = (speedError(ii) - speedErrorLast(ii)) / dt;
				%Store the old value of speed error.
				speedErrorLast(ii)=speedError(ii);
				%Saturate the integrated speed error based on pitch saturation.
				intSpeedError(ii) = min(max(intSpeedError(ii), PitchMin*degRad/PitchControlKI(ii)), PitchMax*degRad/PitchControlKI(ii));
				%Compute the pitch components from the proportional, integral, and derivative parts and sum them.
				pitchP =  PitchControlKP(ii)* speedError(ii); %[rad]
				pitchI =  PitchControlKI(ii)* intSpeedError(ii); %[rad]
				pitchD =  PitchControlKD(ii)* derivSpeedError(ii); %[rad]
				pitchAngleArrayOut(ii)= (pitchP + pitchI + pitchD) / degRad; %[deg]
				pitchAngleArrayOut(ii)= min(max(pitchAngleArrayOut(ii), PitchMin), PitchMax);
                end
                InitializePitchControl2(ii)=true;
        else
          %POWER TRACKING ALGORITHM (KNU2) 
			turbIsGreedy(ii) = false;
			disp(['Turbine is tracking power.'])
			%PITCH POWER REFERENCE CONTROL
                rotorSpeedReference(ii) = min(rotorSpeedReference(ii),rotorSpeedRated);
					if InitializePitchControl2(ii)    
							PitchControlKI(ii) = -Ki(bladePitchArray(ii)*degRad)*gbRatio;
							intSpeedError(ii) = (bladePitchArray(ii)*degRad) / PitchControlKI(ii);
							speedErrorLast(ii)= 0;
						InitializePitchControl2(ii)=false;
					end
				%Compute the gains
				PitchControlKP(ii)= -Kp(bladePitchFiltered(ii)*degRad)*gbRatio;
				PitchControlKI(ii)= -Ki(bladePitchFiltered(ii)*degRad)*gbRatio;
				PitchControlKD(ii)= -Kd(bladePitchFiltered(ii)*degRad)*gbRatio;
				%Compute the low speed shaft speed error.
				speedError(ii)= rotorSpeedArray(ii)- rotorSpeedReference(ii); %[rad/s]
				%Numerically integrate the speed error over time.
				intSpeedError(ii)= intSpeedError(ii)+speedError(ii)*dt;
				%Numerically take the deriviative of speed error w.r.t time.
				derivSpeedError(ii) = (speedError(ii) - speedErrorLast(ii)) / dt;
				%Store the old value of speed error.
				speedErrorLast(ii)=speedError(ii);
				%Saturate the integrated speed error based on pitch saturation.
				intSpeedError(ii) = min(max(intSpeedError(ii), PitchMin*degRad/PitchControlKI(ii)), PitchMax*degRad/PitchControlKI(ii));
				%Compute the pitch components from the proportional, integral, and derivative parts and sum them.
				pitchP =  PitchControlKP(ii)* speedError(ii); %[rad]
				pitchI =  PitchControlKI(ii)* intSpeedError(ii); %[rad]
				pitchD =  PitchControlKD(ii)* derivSpeedError(ii); %[rad]
				pitchAngleArrayOut(ii)= (pitchP + pitchI + pitchD) / degRad; %[deg]
				pitchAngleArrayOut(ii)= min(max(pitchAngleArrayOut(ii), PitchMin), PitchMax);       
			
			% TORQUE CONTROLLER
			genSpeed = rotorSpeedArray(ii) * gbRatio; % in rad/s
            torqueTracking =  powerSetpointTurbCurrent(ii) / (genSpeed*genEfficiency); % Tracking control signal  
			if downRegulationMode==1
                torqueArrayOut(ii)=torqueTracking;
            elseif downRegulationMode==2
                torqueGreedy   = KGen * (genSpeed.^2); % Greedy control signal (Does Kgen is set fot genSpeed or rotSpeed?  
                if (genSpeed > VS_RtGnSp)
                    disp(['Current torque control mode: Region 3.']);
                    torqueOutConv = RatedGenTorque; % Constant torque [also need to change the transition]
                elseif genSpeed< (VS_CtInSp)
                    torqueOutConv=0;
                    disp(['Current torque control mode: Region 1.']);
                elseif genSpeed< (VS_Rgn2Sp)   
                    torqueOutConv= VS_Slope15*( genSpeed - VS_CtInSp);
                    disp(['Current torque control mode: Region 1.1/2.']);
                elseif genSpeed< (VS_TrGnSp)
                    disp(['Current torque control mode: Region 2.']);
                    torqueOutConv = torqueGreedy; % Perfect tracking whenever possible, otherwise fall back on greedy
                else
                    disp(['Current torque control mode: Region 2.1/2.']);
                    torqueOutConv= Region2EndGenTorque + VS_Slope25*( genSpeed - VS_TrGnSp);
                end
                 torqueArrayOut(ii) = min( torqueTracking, torqueOutConv); %see more details in Silva 2022 at TORQUE paper
                if torqueArrayOut(ii)==torqueTracking
                    disp(['Torque tracking has been applied']);
                else
                    disp(['Torque greedy has been applied']);
                end
            else
                disp(['The down-regulation mode is not found.']);
                return
            end
			InitializePitchControl1(ii)=true;
		end
		
		%Saturate using the maximum torque limit
        if (min(torqueArrayOut(ii),VS_MaxTq)==VS_MaxTq)
            torqueArrayOut(ii)=VS_MaxTq;
            disp(['Generator torque reaches maximum!']);
        end 
		
		% RATE LIMITERS
        applyRateLimiterTorque = true;
        if applyRateLimiterTorque
            torqueRateLimit = 15.0e3 * dt;
            deltaTorque = torqueArrayOut(ii) - genTorqueArray(ii);
            deltaTorque = max(min(deltaTorque,torqueRateLimit),-torqueRateLimit);
            torqueArrayOut(ii) = genTorqueArray(ii) + deltaTorque;
        end
        %%{
        applyRateLimiterPitch = true;
        if applyRateLimiterPitch
            pitchRateLimit = 2.5 * dt;
            deltaPitch = pitchAngleArrayOut(ii) - bladePitchArray(ii);
            deltaPitch = max(min(deltaPitch,pitchRateLimit),-pitchRateLimit);
            pitchAngleArrayOut(ii) = bladePitchArray(ii) + deltaPitch;
        end
				
    end    
	
    % Create updated string
    disp([datestr(rem(now,1)) '__    Synthesizing message string.']);
    dataSend = setupZmqSignal(torqueArrayOut,yawAngleArrayOut,pitchAngleArrayOut);
    
    % Send a message (control action) back to SOWFA
    zmqServer.send(dataSend);
	
	 % Save variables
	estimatedThrustArray=[estimatedThrustArray; estimatedThrust];
    varThrustArray=[varThrustArray; varThrust];
	indtrackingThrustVarIntArray=[indtrackingThrustVarIntArray; indtrackingThrustVarInt];
	rotorSpeedReferenceArray = [rotorSpeedReferenceArray; rotorSpeedReference(1)];
    estimatedWindSpeedArray = [estimatedWindSpeedArray; arrayfun(@(ii) WSE{ii}.windSpeed,1:nTurbs)];
    powerSetpointTurbArray  = [powerSetpointTurbArray; powerSetpointTurbCurrent];
    powerAvailableTurbArray  = [powerAvailableTurbArray; powerAvailableTurb];
    up = [up; powerSetpointVariationPowerControl];
    powerSetpointTurbArrayNC= [powerSetpointTurbArrayNC; powerSetpointTurbNC];
    ut = [ut; powerSetpointVariationThrustControl];
    powerSetpointTurbArrayNTC = [powerSetpointTurbArrayNTC; powerSetpointTurbNTC];
    sumNonSatutArray=[sumNonSatutArray;sumNonSatut];
	powerSetpointTurbArrayLC=[powerSetpointTurbArrayLC; powerSetpointTurbLC];
	
	if ~rem(currentTime,10)
        save('workspace.mat')
    end	
end

% Close connection
zmqServer.disconnect()

function [dataOut] = setupZmqSignal(torqueSignals,yawAngles,pitchAngles)
	dataOut = [];
    for i = 1:length(yawAngles)
        dataOut = [dataOut torqueSignals(i) yawAngles(i) pitchAngles(i)];
    end
end