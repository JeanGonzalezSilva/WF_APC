%Edited by Jean Gonzalez Silva 18/07/2022
%DTU 10MW
clear all
addpath('C:\Users\jgonzalessilva\Desktop\SOWFA_tools-master\SOWFA_tools-master\readTurbineOutput')
load('workspace.mat')

%initialization
rho=1.225;
Rr=89.2;
Cpmax=0.5672;
ratedPower=10e6;
%rotSpeedRated=0.87060; %DRC
rotSpeedRated = 0.8921; %ROSCO

[time,power]=readOOPTdata('../turbineOutput/20000/powerGenerator');
power = power / 1.225;

%xlims = [time(1) time(end)];
%xlims = [280 400];

%%{
figure(1)
ii = 1;
Nmin = min([size(time,1) size(powerSetpointTurbArray,1)]);
plot(rem(time(1:Nmin),1e4),powerSetpointTurbArray(1:Nmin,ii),'-g')
hold on
plot(rem(time(1:Nmin),1e4),powerSetpointTurbArrayNC(1:Nmin,ii))
plot(rem(time,1e4),power(:,ii), '-.b')
hold off
ylabel('P^{WT} (W)')
xlim([200,1000])
%ylim([0,6e6])
box on
grid on
legend('P_{ref}^{WT_i}','P_{ref, no control}^{WT_i}','P_{meas}^{WT_i}','Location','best')
%}

powertotal=sum(power,2);
powerSetpointFarm=sum(powerSetpointTurbArray,2);
figure(2)
Nmin= min([size(time,1) size(powerSetpointTurbArray,1)]);
plot(rem(time(1:Nmin),1e4),powerSetpointFarm(1:Nmin),'-g')
hold on
plot(rem(time(1:Nmin),1e4),powerSetpointFarmArray(1:Nmin))
plot(rem(time(1:Nmin),1e4),powertotal(1:Nmin), '-.b')
hold off
ylabel('P^{WF} (W)')
xlim([200,1000])
%ylim([0,6e6])
box on
grid on
legend('P_{ref}^{WF}','P_{ref, no control}^{WF}','P_{meas}^{WF}','Location','best')


%%{
[time,torqueGen]=readOOPTdata('../turbineOutput/20000/torqueGen');
[time,pitch]=readOOPTdata('../turbineOutput/20000/pitch');
%[time,pitchout]=readPitchData('../turbineOutput/20000/pitch');
%for ii=1:nTurbs
%pitch(:,ii)=pitchout{ii}(:,1);
%end
[time,thrust]=readOOPTdata('../turbineOutput/20000/thrust');
xlims = [100 1000];
figure(3)
    subplot(3,1,1)
    hold all
    plot(rem(time,1e4),torqueGen(:,1))
    plot(rem(time,1e4),torqueGen(:,10),'--')
    plot(rem(time,1e4),torqueGen(:,18),'-.')
    plot(rem(time,1e4),torqueGen(:,27),':')
    ylabel('Generator torque (Nm)')
    legend('G.torque 1','G.torque 10','G.torque 18','G.torque 27')
    xlim(xlims)

    subplot(3,1,2)
    hold all
    plot(rem(time,1e4),pitch(:,1))
    plot(rem(time,1e4),pitch(:,10),'--')
    plot(rem(time,1e4),pitch(:,18),'-.')
    plot(rem(time,1e4),pitch(:,27),':')
    ylabel('Pitch (degree)')
    legend('Pitch 1','Pitch 10','Pitch 18','Pitch 27')
    xlim(xlims)

    subplot(3,1,3)
    hold all
    plot(rem(time,1e4),thrust(:,1))
    plot(rem(time,1e4),thrust(:,10),'--')
    plot(rem(time,1e4),thrust(:,18),'-.')
    plot(rem(time,1e4),thrust(:,27),':')
    ylabel('Thrust (N)')
    legend('Thrust 1','Thrust 10','Thrust 18','Thrust 27')
    %ylim([0 1.6e6])
    xlim(xlims)
    box on
    grid on

%}

%%{
 rpmRadSec=2*pi()/60;
 %[time,rotSpeed]=readOOPTdata('../turbineOutput/0/rotSpeed');
 [time,rotSpeedFiltered]=readOOPTdata('../turbineOutput/20000/rotSpeedFiltered');
rotSpeedFiltered=rotSpeedFiltered/9.5493;

    figure(4)
   plot(time, rotSpeedFiltered(:,1))
   hold all
   plot(time(1:Nmin), rotorSpeedReferenceArray(1:Nmin))
   %plot(time, rotSpeedFiltered(:,2),'--')
   %plot(time, rotSpeedFiltered(:,3),'-.')
   plot(time, rotSpeedRated*ones(length(time),1),':')
   legend('Rotor speed measure','Rotor speed reference','Rated rotor speed')
%}

%%
%%{
figure(5)
column=4; 
line=8; 
for ii = 1:nTurbs
        subplot(column,line,ii)
        hold all
        Nmin = min([size(time,1) size(powerSetpointTurbArray,1)]);
        plot(rem(time(1:Nmin),1e4),powerSetpointTurbArray(1:Nmin,ii),'-g')
		plot(rem(time(1:Nmin),1e4),powerSetpointTurbArrayNC(1:Nmin,ii))
        plot(rem(time,1e4),power(:,ii), '-.b')
        ylabel(['P^{WT_i} (W) of i=' num2str(ii)])
        xlim([200,1000])
        %ylim([0,10e6])
        box on
        grid on
end
hold off
legend('P_{ref}^{WT_i}','P_{ref, no control}^{WT_i}','P_{meas}^{WT_i}','Location','best')
%}
%%