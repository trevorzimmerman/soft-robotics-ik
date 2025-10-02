clc; clear; close all;

% ============================================================
% Parameters and symbolic setup
% ============================================================
% Define symbolic joint variables and link lengths
syms t1 t2 T_total_sym
theta = [t1; t2];
link_len = [1, 1];

% Radii of curvature for each joint
radii = [link_len(1)/t1, link_len(2)/t2];

% Initialize transformation matrix
T_total_sym = eye(3);

% ============================================================
% Forward kinematics
% ============================================================
% Compute homogeneous transformations and end-effector position
for i = 1:2
    T(i,:,:) = [cos(theta(i)) -sin(theta(i)) radii(i)*sin(theta(i));
                sin(theta(i))  cos(theta(i)) radii(i)*(1-cos(theta(i)));
                0              0             1];
    T_total_sym = T_total_sym * squeeze(T(i,:,:));
    transforms{i} = T_total_sym;
    pos(:,i) = simplify(T_total_sym * [0;0;1]);
end

% Extract end-effector position (x, y)
ee_pos = pos(1:2,2);

% ============================================================
% Jacobian calculation
% ============================================================
% Derive Jacobian of the end-effector position 
% with respect to joint variables
J = jacobian(ee_pos, [t1, t2]);

% ============================================================
% Trajectory definition
% ============================================================
% Elliptical path
T_total = 6;
dt = 0.2;
t = 0:dt:T_total;
x_ellipse = 0.2 * cos(2*pi*t/T_total) + 0.2;
y_ellipse = 0.1 * sin(2*pi*t/T_total) + 1.0;

% Horizontal zig-zag path
x_line = 0.4:(0.4/16):0.8;
x_rev = x_line; x_rev([1 end]) = []; x_rev = flip(x_rev);
x_line = [x_line x_rev]; x_line(1) = [];
y_line = ones(1, length(x_line));

% Combined trajectory
x_full = [x_ellipse x_line];
y_full = [y_ellipse y_line];
t = 0:2/60:2;
Xd = [x_full', y_full'];

% ============================================================
% Inverse kinematics
% ============================================================
% Iteratively solve joint angles using Jacobian-based updates
Qd = [1;1]; % initial guess
for i = 1:length(t)+1
    err = 1;
    while err > 1e-5
        J_eval = eval(subs(J, {t1,t2}, Qd'));
        ee_eval = eval(subs(ee_pos, {t1,t2}, Qd'));
        Qd = inv(J_eval) * (Xd(i,:)' - ee_eval) + Qd;
        ee_eval = eval(subs(ee_pos, {t1,t2}, Qd'));
        err = norm(abs(Xd(i,:)' - ee_eval));
    end
    Q(i,:) = Qd;
end

% ============================================================
% Animation
% ============================================================
% Visualize the arm following the trajectory
fig = figure;
frame_idx = 1;

for i = 1:length(t)+1
    beta1 = 0:Q(i,1)/100:Q(i,1);
    beta2 = 0:Q(i,2)/100:Q(i,2);
    r1 = link_len(1)/beta1(end);
    r2 = link_len(2)/beta2(end);

    x1 = r1 * sin(beta1);
    y1 = r1 * (1 - cos(beta1));
    x2 = r2 * sin(beta2);
    y2 = r2 * (1 - cos(beta2));

    for j = 1:length(beta1)
        X2(j,:) = eval(subs(transforms{1},{t1},beta1(end))) * ...
                  [x2(j); y2(j); 1];
    end

    plot(X2(:,1), X2(:,2), 'b', x1, y1, 'r', x_full, y_full, 'k')
    axis([-0.2 1.2 -0.2 1.2])
    title('Soft Robotic Arm Motion Along Elliptical and Zig-Zag Paths')
    xlabel('X', 'FontWeight', 'bold')
    ylabel('Y', 'FontWeight', 'bold')
    drawnow

    frame = getframe(fig);
    im{frame_idx} = frame2im(frame);
    frame_idx = frame_idx + 1;
end

% ============================================================
% Export GIF
% ============================================================
% Save the generated animation to file
[folder, ~, ~] = fileparts(mfilename('fullpath'));
gifFolder = fullfile(folder, 'gif');
if ~exist(gifFolder, 'dir')
    mkdir(gifFolder);
end
filename = fullfile(gifFolder, 'jacobian_ik_2link_arm.gif');

for i = 1:frame_idx-1
    [A,map] = rgb2ind(im{i},256);
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.05);
    else
        imwrite(A,map,filename,'gif','WriteMode','append', ...
                'DelayTime',0.05);
    end
end
close(fig);