clc; clear; close all;

% ============================================================
% Load trained ANFIS models
% ============================================================
% anfis1 and anfis2 are Sugeno FIS models used for inverse kinematics
[folder, ~, ~] = fileparts(mfilename('fullpath'));
load(fullfile(folder,'data','Anfis1.mat'))
load(fullfile(folder,'data','Anfis2.mat'))

% ============================================================
% Trajectory definition
% ============================================================
% Elliptical path
T = 6;
dt = 0.2;
t = 0:dt:T;
xEllipse = 0.25*cos(2*pi*t/T) + 0.8;
yEllipse = 0.15*sin(2*pi*t/T) + 3.44;

% Horizontal line path (right then left)
xLine = 1.05:((1.5-1.05)/15):1.5;
xRev = xLine; xRev(1) = []; xRev(end) = []; xRev = flip(xRev);
xLine = [xLine xRev];
yLine = 3.44 * ones(size(xLine));

% Combined trajectory
xFull = [xEllipse xLine];
yFull = [yEllipse yLine];
t = 0:2/60:2;
XY = [xFull' yFull'];

% ============================================================
% Inverse kinematics with ANFIS
% ============================================================
% Predict joint angles from Cartesian trajectory
theta1Pred = evalfis(anfis1, XY);
theta2Pred = evalfis(anfis2, XY);

% ============================================================
% Animation
% ============================================================
% Simulate 2-DOF soft parallel robot following trajectory
f = figure;
frameIdx = 1;
tLayout = tiledlayout(1,2,"TileSpacing","compact");

for i = 1:length(t)
    % Extract predicted angles
    theta1 = theta1Pred(i);
    theta2 = theta2Pred(i);

    % Robot parameters
    L1 = 2.3; L2 = 1.9; d = 2;
    r1 = L1/theta1; r2 = L1/theta2;

    % Curved link shapes
    x1 = r1 * sin(theta1);
    y1 = r1 * (1 - cos(theta1));
    x2 = r2 * sin(theta2);
    y2 = r2 * (1 - cos(theta2));

    % End-effector position
    p1 = [-r1*(1-cos(theta1)); r1*sin(theta1)];
    p2 = [d-r2*(1-cos(theta2)); r2*sin(theta2)];
    phi1 = atan((p2(2)-p1(2))/(p2(1)-p1(1))) + ...
           acos(((p2(1)-p1(1))^2 + (p2(2)-p1(2))^2)^0.5/(2*L2));
    phi2 = pi + atan((p2(2)-p1(2))/(p2(1)-p1(1))) - ...
           acos(((p2(1)-p1(1))^2 + (p2(2)-p1(2))^2)^0.5/(2*L2));

    b1 = [cos(pi/2) -sin(pi/2) 0      
          sin(pi/2)  cos(pi/2) 0
          0          0         1];
    b2 = [sin(phi1) cos(phi1) x1
         -cos(phi1) sin(phi1) y1
          0         0         1];
    b3 = [cos(pi/2) -sin(pi/2) d
          sin(pi/2)  cos(pi/2) 0 
          0          0         1];
    b4 = [sin(phi2) cos(phi2) x2
         -cos(phi2) sin(phi2) y2
          0         0         1];

    E = b1*b2*[L2;0;1];              

    % Intermediate link points
    theta1Vals = linspace(0,theta1,10);
    theta2Vals = linspace(0,theta2,10);
    x1 = r1 * sin(theta1Vals);
    y1 = r1 * (1 - cos(theta1Vals));
    x2 = r2 * sin(theta2Vals);
    y2 = r2 * (1 - cos(theta2Vals));

    % Plot workspace motion
    nexttile(1); cla; hold on
    p1 = b1 * [x1; y1; ones(size(theta1Vals))];
    plot(p1(1,:),p1(2,:),'r','LineWidth',1)
    plot([p1(1,end),E(1)],[p1(2,end),E(2)],'b','LineWidth',1)

    p2 = b3 * [x2; y2; ones(size(theta2Vals))];
    plot(p2(1,:),p2(2,:),'r','LineWidth',1)
    plot([p2(1,end),E(1)],[p2(2,end),E(2)],'b','LineWidth',1)

    axis([-1 3 0 4])
    plot(xFull,yFull,'k','LineWidth',1)
    title('Soft Parallel Robot: Trajectory Tracking')
    xlabel('X'); ylabel('Y');
    drawnow

    % Plot joint motion vs time
    nexttile(2); cla; hold on
    plot(t,180*theta1Pred/pi,'r',t,180*theta2Pred/pi,'--r')
    plot(t(i),180*theta1Pred(i)/pi,'*k',t(i),180*theta2Pred(i)/pi,'*k')
    legend('Left Joint','Right Joint','Location','northeast')
    title('Joint Angles Over Time (ANFIS)')
    xlabel('Time (s)','FontWeight','bold')
    ylabel(['Angle (\circ)'],'FontWeight','bold')

    % Store frame for GIF
    frame = getframe(f);
    im{frameIdx} = frame2im(frame);
    frameIdx = frameIdx + 1;
end

% ============================================================
% Export GIF
% ============================================================
% Save animation to file
[folder, ~, ~] = fileparts(mfilename('fullpath'));
gifFolder = fullfile(folder, 'gif');
if ~exist(gifFolder, 'dir')
    mkdir(gifFolder);
end
filename = fullfile(gifFolder, 'anfis_ik_soft_parallel_arm.gif');

for i = 1:frameIdx-1
    [A,map] = rgb2ind(im{i},256);
    if i == 1
        imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",0.033);
    else
        imwrite(A,map,filename,"gif","WriteMode","append", ...
                "DelayTime",0.033);
    end
end
close(f)