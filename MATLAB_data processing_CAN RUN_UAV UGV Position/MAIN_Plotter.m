%Autonomous Coupling of UAV and UGV: Position Plotter
clear all
close all
clc
load('rawData.mat')

%% 1. Follow // Aero v Ground // Ground v Waypoints

% Extract desired inteval from raw data - plot to check
Af = A1RAW(900:1980,:); %Extract following data from set 1
% figure()
% plot3(Af(:,1), Af(:,2), Af(:,3))
% title('Aero Path xyz')
% xlabel('x'); ylabel('y'); zlabel('z')
% xlim([-3,3]); ylim([-12,-4]); zlim([0,2])

% Extract desired inteval from raw data - plot to check
Gf = G1RAW(1200:1875,:); %1875
% figure()
% plot(Gf(:,1), Gf(:,2))
% title('Ground Path xy')
% xlabel('x'); ylabel('y');
% xlim([-0.5,2.5]); ylim([-12,-4.5]);

% Round timestamps for to sync data points for error calculation
AfRound = [Af(:,1:3), round(Af(:,4),3)];
GfRound = [G1RAW(900:1876,1:3), round(G1RAW(900:1876,4),3)]; %3342
% Initialize synced intervals
Afsync = [];
Gfsync = [];
sync = [];
% loop through interval matching time stamps and append synced interval
for i = 1:length(AfRound(:,4))
    sync = find(AfRound(i,4) == GfRound(:,4));
    if(sync ~= 0)
        for j = 1:length(sync)
            Afsync = [Afsync; AfRound(i,:)];
            Gfsync = [Gfsync; GfRound(sync(j),:)];
        end
    end
end

% Aero Tracking Ground Tracking Waypoints
fwaypoints = [Gfsync(1,1),Gfsync(1,2);2,-9;0,-7;0,-3];
% Ground obstacle
fobstacle = [-0.5,-5;-0.25,-5;0,-5;0.25,-5;0.5,-5];

% plot synced position data
figure('Position', [500 50 500 775])
plot(Afsync(:,1),Afsync(:,2),'color',[0.8500 0.3250 0.0980])
hold on 
plot(Gfsync(:,1),Gfsync(:,2),'color',[0 0.4470 0.7410])
set(gca,'xtick',[-1:1:3])
xlim([-1,3]); ylim([-11.5,-2.5]);
grid on
pbaspect([1, 9/4, 1])
% plot waypoints with error margin
plot(fwaypoints(2:end,1),fwaypoints(2:end,2),'o','color',[0.5 0.5 0.5],'MarkerSize',8)
% plot desired path
plot(fwaypoints(:,1),fwaypoints(:,2),'--','color',[0.5 0.5 0.5],'HandleVisibility','off')
% plot obstacle
scatter(fobstacle(:,1),fobstacle(:,2),'rs')
plot(fobstacle(3,1),fobstacle(3,2),'o','HandleVisibility','off','MarkerSize',32,'color',[1 0.5 0.5])
legend('UAV','UGV','Waypoints','Obstacle')
xlabel('x (m)')
ylabel('y (m)')
% plot initial and final positions
plot(Afsync(1,1),Afsync(1,2),'o','color',[0.8500 0.3250 0.0980],'HandleVisibility','off')
plot(Gfsync(1,1),Gfsync(1,2),'o','color',[0 0.4470 0.7410],'HandleVisibility','off')
plot(Afsync(end,1),Afsync(end,2),'x','color',[0.8500 0.3250 0.0980],'HandleVisibility','off')
plot(Gfsync(end,1),Gfsync(end,2),'x','color',[0 0.4470 0.7410],'HandleVisibility','off')

%% 2. Couple // Aero v Ground

Ac = A1RAW(1890:3150,:); % Extract coupling data from set 1
gFinal = Gfsync(end,:); % Match stationary position to previous final position
% Waypoint positions
cwaypoints = [Ac(1,2),Ac(1,3);
                gFinal(2),gFinal(3)+1;
                gFinal(2),gFinal(3)+0.5;
                gFinal(2),gFinal(3)+0.2;
                gFinal(2),gFinal(3)+0.12];
% Obstacle position
cobstacle = [-5,0;-5,0.1;-5,0.2;-5,0.3;-5,0.4];

% Plot position data
figure('Position', [500 50 500 775])
plot(Ac(:,2),Ac(:,3),'color',[0.8500 0.3250 0.0980])
xlabel('y (m)'); ylabel('z (m)');
xlim([-5.8,-5]); ylim([0,1.8])
set(gca,'xtick',[-5.8:0.2:-5])
set(gca,'ytick',[0:0.2:1.8])
grid on
pbaspect([1 1.8/.8 1])
hold on
% plot initial position
plot(gFinal(2),gFinal(3),'o','color',[0 0.4470 0.7410])
% plot waypoints with error margin
plot(cwaypoints(2:3,1),cwaypoints(2:3,2),'o','MarkerSize',53,'color',[0.5 0.5 0.5])
plot(cwaypoints(4,1),cwaypoints(4,2),'o','MarkerSize',27,'color',[0.5 0.5 0.5],'HandleVisibility','off')
plot(cwaypoints(5,1),cwaypoints(5,2),'o','MarkerSize',8,'color',[0.5 0.5 0.5],'HandleVisibility','off')
% plot obstacle
plot(cobstacle(:,1),cobstacle(:,2),'rs')
% plot desired path
plot(cwaypoints(:,1),cwaypoints(:,2),'--','color',[0.5 0.5 0.5],'HandleVisibility','off')
legend('UAV','UGV','Waypoints','Obstacle')
% plot initial and final positions
plot(Ac(1,2),Ac(1,3),'o','color',[0.8500 0.3250 0.0980],'HandleVisibility','off')
plot(Ac(end,2),Ac(end,3),'x','color',[0.8500 0.3250 0.0980],'HandleVisibility','off')
%% 3. Lift // Aero v Waypoints

Al = A2RAW(1940:2150,:); %Extract lifting data from set 2
% obstacle position
lobstacle = [-5,0;-5,0.125;-5,0.25];
% waypoint position
lwaypoints = [Al(1,2),Al(1,3);gFinal(2),gFinal(3)+0.6;-4.5,0.5;-4.5,0.2];

% plot position data
figure('Position', [500 50 700 500])
plot(Al(:,2), Al(:,3),'color',[0.8500 0.3250 0.0980])
xlabel('y (m)'); ylabel('z (m)')
xlim([-5.5,-4.25]); ylim([0,1])
set(gca,'xtick',[-5.5:0.25:-4.25])
set(gca,'ytick',[0:0.25:1])
grid on
pbaspect([5 4 1])
hold on
% plot waypoints with error margin
plot(lwaypoints(2:end,1),lwaypoints(2:end,2),'o','MarkerSize',123,'color',[0.5 0.5 0.5])
% plot desired path
plot(lwaypoints(:,1),lwaypoints(:,2),'--','color',[0.5 0.5 0.5],'HandleVisibility','off')
% plot obstacle
plot(lobstacle(:,1),lobstacle(:,2),'rs')
% plot initial and final positions
plot(Al(1,2),Al(1,3),'o','color',[0.8500 0.3250 0.0980],'HandleVisibility','off')
plot(Al(end,2),Al(end,3),'x','color',[0.8500 0.3250 0.0980],'HandleVisibility','off')
legend('UAV','Waypoints','Obstacle')

%% 4. Rest // Ground v Waypoints

Gr = G3RAW(2333:3025,:); %Extract resting data from set 3 %519
% waypoint position
rwaypoints = [0,-3;0,-2;1,-1;1,0];
% obstacle position
robstacle = [2,-11;2,-9;0,-7;0,-3];

% plot position data
figure('Position', [500 50 500 775])
plot(Gr(:,1), Gr(:,2),'color',[0 0.4470 0.7410])
grid on
pbaspect([2 4 1])
xlabel('x (m)'); ylabel('y (m)');
xlim([-0.5,1.5]); ylim([-3.5,0.5]);
hold on
% plot waypoints
plot(rwaypoints(2:end,1),rwaypoints(2:end,2),'o','color',[0.5 0.5 0.5],'MarkerSize',17)
% plot desired path
plot(rwaypoints(:,1),rwaypoints(:,2),'--','color',[0.5 0.5 0.5],'HandleVisibility','off')
legend('UGV','Waypoints')
% plot initial and final positions
plot(Gr(1,1),Gr(1,2),'o','color',[0 0.4470 0.7410],'HandleVisibility','off')
plot(Gr(end,1),Gr(end,2),'x','color',[0 0.4470 0.7410],'HandleVisibility','off')
