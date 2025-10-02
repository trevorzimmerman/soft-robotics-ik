% ============================================================
% Soft Parallel Robot GIF Animation (vpasolve angles)
% ============================================================
% In this script, the robot follows precomputed joint angle
% trajectories loaded from data.mat. The robot is visualized
% from multiple views using a tiled layout, and a GIF is saved.
% The first, third, and fourth tiles show the same plot with
% different views. Only the joint angles are updated each frame.
clc; clear; close all;

% ============================================================
% Symbolic variables for joint angles and platform coordinates
% ============================================================
syms theta1 theta2 theta3 x1 x2 x3

% ============================================================
% Load precomputed joint angles from vpasolve
% data.mat contains 'data', a 3xN array of angles in degrees:
% data(1,:) = theta1, data(2,:) = theta2, data(3,:) = theta3
% ============================================================
[folder, ~, ~] = fileparts(mfilename('fullpath'));
load(fullfile(folder,'data','data.mat'))

% ============================================================
% Circular trajectory parameters
% ============================================================
T = 3;           % total time for circular motion
dt = 0.1;        % time step
t = 0:dt:T;

% Circle in XY-plane at constant Z
x = 5*cos(2*pi*t/T);
y = 5*sin(2*pi*t/T);
z = -45*ones(1,length(x));

% ============================================================
% Vertical trajectory parameters
% ============================================================
xV = x(1)*ones(1,floor(length(x)/2)); % X fixed
yV = y(1)*ones(1,floor(length(y)/2)); % Y fixed

% Downward and upward motion along Z
zV_down = -26:(z(1)+26)/14:z(1);    % down
zV_up = z(1):(-26-z(1))/14:-26;     % up

% ============================================================
% Concatenate trajectories: down + circle + up
% ============================================================
x = cat(2,xV,x,xV);
y = cat(2,yV,y,yV);
z = cat(2,zV_down,z,zV_up);

% Update total time vector for plotting
T = 6;
t = 0:dt:T;

% Frame counter for GIF
g = 1;

% Create figure with 2x2 tiled layout
f3 = figure; 
tL = tiledlayout(2,2,"TileSpacing","compact");

% ============================================================
% Loop over trajectory points
% ============================================================
for i = 1:length(x)
    % Current desired input position of the moving platform
    x_in = [x(i), y(i), z(i)];

    % ---------------------------------------------------------------------
    % Fixed platform setup
    % ---------------------------------------------------------------------
    X = [x1;x2;x3];       
    p = X;               % desired position of moving platform
    theta = [theta1;theta2;theta3];

    sb = 60;             % side of base triangle
    wb = sb*3^.5/6;      
    ub = sb*3^.5/3;      
    Lb = 20;             % length of lower link
    Lp = 30;             % length of upper link

    % Revolute joints on the base
    B(:,1) = [0;-wb;0];
    B(:,2) = [wb*3^.5/2;wb/2;0];
    B(:,3) = [-wb*3^.5/2;wb/2;0];

    % Base triangle (for plotting)
    b(:,1) = [sb/2;-wb;0];        
    b(:,2) = [0;ub;0];
    b(:,3) = [-sb/2;-wb;0];

    % ---------------------------------------------------------------------
    % Soft links description
    % ---------------------------------------------------------------------
    r1_ = Lb/theta(1);
    r2_ = Lb/theta(2);
    r3_ = Lb/theta(3);

    L(:,1) = r1_*[0; ...
                  -(1-cos(theta(1))); ...
                  -sin(theta(1))];
    L(:,2) = r2_*[(1-cos(theta(2)))*3^.5/2; ...
                  (1-cos(theta(2)))/2; ...
                  -sin(theta(2))];
    L(:,3) = r3_*[-(1-cos(theta(3)))*3^.5/2; ...
                  (1-cos(theta(3)))/2; ...
                  -sin(theta(3))];

    % ---------------------------------------------------------------------
    % Moving platform (triangular)
    % ---------------------------------------------------------------------
    sp = 10;           
    wp = sp*3^.5/6;    
    up = sp*3^.5/3;    

    P(:,1) = [0;-up;0];   % corners of moving platform in local frame
    P(:,2) = [sp/2;wp;0];
    P(:,3) = [-sp/2;wp;0];

    % Global coordinates of platform corners and constraints
    for j = 1:3
        PPP(:,j) = p+P(:,j);              % platform corners in global
        f(:,j) = p+P(:,j)-B(:,j)-L(:,j);  % link vector
        ff(j) = f(:,j)'*f(:,j) - Lp^2;    % squared length constraint
    end

    % ---------------------------------------------------------------------
    % Solve inverse kinematics using vpasolve
    % ---------------------------------------------------------------------
    theta0 = [1,1,1];
    eqn = eval(subs(ff,[x1,x2,x3],x_in))==0;
    a = vpasolve(eqn,theta,theta0);

    % Extract solved joint angles
    out = [a.theta1,a.theta2,a.theta3];
    A1 = double(a.theta1);
    A2 = double(a.theta2);
    A3 = double(a.theta3);

    PP = eval(subs(PPP,[x1,x2,x3],x_in));

    % ---------------------------------------------------------------------
    % Beta angles for plotting soft links
    % ---------------------------------------------------------------------
    beta1_ = 0:A1/100:A1; beta1_(1) = 0.001;
    beta2_ = 0:A2/100:A2; beta2_(1) = 0.001;
    beta3_ = 0:A3/100:A3; beta3_(1) = 0.001;

    % Soft link 1 coordinates
    L1_ = eval(subs(L(:,1), theta(1), beta1_(end))) + B(:,1);
    r1_ = Lb/beta1_(end);
    y1_ = -r1_*(1-cos(beta1_)) + B(2,1);
    z1_ = -r1_*sin(beta1_);
    x1_ = zeros(1,length(y1_));

    % Soft link 2 coordinates
    L2_ = eval(subs(L(:,2), theta(2), beta2_(end))) + B(:,2);
    r2_ = Lb/beta2_(end);
    x2_ = r2_*((1-cos(beta2_))*3^.5/2) + B(1,2);
    y2_ = r2_*(1-cos(beta2_))/2 + B(2,2);
    z2_ = -r2_*sin(beta2_);

    % Soft link 3 coordinates
    L3_ = eval(subs(L(:,3), theta(3), beta3_(end))) + B(:,3);
    r3_ = Lb/beta3_(end);
    x3_ = -r3_*((1-cos(beta3_))*3^.5/2) + B(1,3);
    y3_ = r3_*(1-cos(beta3_))/2 + B(2,3);
    z3_ = -r3_*sin(beta3_);

    % ---------------------------------------------------------------------
    % Base triangle circle coordinates
    % ---------------------------------------------------------------------
    bT = 360;
    bt = 0:359;
    bx = wb*cos(2*pi*bt/bT);
    by = wb*sin(2*pi*bt/bT);
    bz = zeros(length(bx));

    % ============================================================
    % Tiled plot: multiple views
    % ============================================================
    % Tile 1: main trajectory
    nexttile(1); cla; hold on
    plot3(x,y,z,'m')
    axis([-30 30 -35 20 -45 0]);
    plot3(B(1,:),B(2,:),B(3,:),'k*')                    % base joints
    plot3(x_in(1),x_in(2),x_in(3),'rx')                 % desired platform
    plot3(bx,by,bz,'c-')                                % base circle
    plot3(PP(1,1:3),PP(2,1:3),PP(3,1:3),'*b-')          % moving platform
    plot3(PP(1,[3,1]),PP(2,[3,1]),PP(3,[3,1]),'*b-')

    for j = 1:3
        LL(:,j) = subs(L(:,j),theta(j),out(j)) + B(:,j);
        if j==1
            plot3(x1_,y1_,z1_,'r-')
        elseif j==2
            plot3(x2_,y2_,z2_,'g-')
        else
            plot3(x3_,y3_,z3_,'b-')
        end
        plot3([LL(1,j),PP(1,j)],[LL(2,j),PP(2,j)],...
              [LL(3,j),PP(3,j)],'*k-')
    end
    v = [-30 -30 25];
    view(v)
    xlabel('x'); ylabel('y'); zlabel('z');
    title('All Trajectories'); grid on

    % Tile 2: joint angles vs time
    nexttile(2); cla; hold on
    plot(t,data(1,:),'-r',t,data(2,:),'-g',t,data(3,:),'-b')
    plot(t(g),A1*180/pi,'*r',t(g),A2*180/pi,'*g',t(g),A3*180/pi,'*b')
    legend('Joint 1','Joint 2','Joint 3','Location','north');
    title('vpasolve: Motion Angle VS Time')
    xlabel('Time (s)','FontWeight','bold')
    ylabel(['Angle (\circ)'],'FontWeight','bold')

    % Tile 3: side view
    nexttile(3); cla; hold on
    plot3(x,y,z,'m')
    axis([-30 30 -35 20 -45 0]);
    plot3(B(1,:),B(2,:),B(3,:),'k*')
    plot3(x_in(1),x_in(2),x_in(3),'rx')
    plot3(bx,by,bz,'c-')
    plot3(PP(1,1:3),PP(2,1:3),PP(3,1:3),'*b-')
    plot3(PP(1,[3,1]),PP(2,[3,1]),PP(3,[3,1]),'*b-')

    for j = 1:3
        LL(:,j) = subs(L(:,j),theta(j),out(j)) + B(:,j);
        if j==1
            plot3(x1_,y1_,z1_,'r-')
        elseif j==2
            plot3(x2_,y2_,z2_,'g-')
        else
            plot3(x3_,y3_,z3_,'b-')
        end
        plot3([LL(1,j),PP(1,j)],[LL(2,j),PP(2,j)],...
              [LL(3,j),PP(3,j)],'*k-')
    end
    v = [0 -30 0];
    view(v)
    xlabel('x'); ylabel('y'); zlabel('z');
    title('Side View'); grid on

    % Tile 4: top view
    nexttile(4); cla; hold on
    plot3(x,y,z,'m')
    axis([-30 30 -35 20 -45 0]);
    plot3(B(1,:),B(2,:),B(3,:),'k*')
    plot3(x_in(1),x_in(2),x_in(3),'rx')
    plot3(bx,by,bz,'c-')
    plot3(PP(1,1:3),PP(2,1:3),PP(3,1:3),'*b-')
    plot3(PP(1,[3,1]),PP(2,[3,1]),PP(3,[3,1]),'*b-')

    for j = 1:3
        LL(:,j) = subs(L(:,j),theta(j),out(j)) + B(:,j);
        if j==1
            plot3(x1_,y1_,z1_,'r-')
        elseif j==2
            plot3(x2_,y2_,z2_,'g-')
        else
            plot3(x3_,y3_,z3_,'b-')
        end
        plot3([LL(1,j),PP(1,j)],[LL(2,j),PP(2,j)],...
              [LL(3,j),PP(3,j)],'*k-')
    end
    v = [0 0 35];
    view(v)
    xlabel('x'); ylabel('y'); zlabel('z');
    title('Top View'); grid on

    % ============================================================
    % Capture frame for GIF
    % ============================================================
    frame = getframe(f3);
    im{g} = frame2im(frame);
    g = g + 1;
end

% ============================================================
% Create GIF
% ============================================================
[folder, ~, ~] = fileparts(mfilename('fullpath'));
gifFolder = fullfile(folder, 'gif');
if ~exist(gifFolder, 'dir')
    mkdir(gifFolder);
end
filename = fullfile(gifFolder, 'vpasolve_ik_3link_parallel_robot.gif');

for h = 1:g-1
    [A,map] = rgb2ind(im{h},256);
    if h==1
        imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",0.033);
    else
        imwrite(A,map,filename,"gif","WriteMode","append","DelayTime", ...
                0.033);
    end
end
close(f3)