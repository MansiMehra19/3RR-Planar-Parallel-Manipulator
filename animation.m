% 3-3R Planar Parallel Manipulator Animation
clear; clc; close all;

% Parameters
L = 20;                      % Length of each link
s_base = 50;                % Side length of base triangle
s_platform = 10;            % Side length of platform triangle
frames = 100;               % Animation frames
theta_motion = linspace(0, pi/3, frames); % Platform rotation angles

% Base triangle points (fixed)
angle = (0:2)*2*pi/3 + pi/6;
B = s_base/2 * [cos(angle)', sin(angle)'];

% Plot setup
figure;
axis equal;
axis([-50 50 -50 50]);
grid on;

% Animation loop
for k = 1:frames
    cla;
    hold on;

    % Moving platform center (circular path)
    phi = theta_motion(k);         % platform rotation
    cx = 10 * cos(phi);            % x of center
    cy = 10 * sin(phi);            % y of center
    platform_center = [cx, cy];

    % Platform triangle vertices (rotated & translated)
    angle_m = (0:2)*2*pi/3 + phi;
    P = platform_center + s_platform/2 * [cos(angle_m)', sin(angle_m)'];

    % Plot base and platform
    plot(B([1:3 1],1), B([1:3 1],2), 'k-', 'LineWidth', 2); % Base
    plot(P([1:3 1],1), P([1:3 1],2), 'b-', 'LineWidth', 2); % Platform

    % For each arm
    for i = 1:3
        % Solve inverse kinematics: find intermediate joint Ji
        d = P(i,:) - B(i,:);     % Vector from Bi to Pi
        D = norm(d);

        % Check for reachability
        if D > 2*L
            warning('Target too far at frame %d', k);
            continue;
        end

        % Find joint point using geometry (2-link mechanism)
        a = D / 2;
        h = sqrt(L^2 - a^2);
        m = (B(i,:) + P(i,:)) / 2;        % midpoint
        perp = [-d(2), d(1)] / D;         % Perpendicular vector
        J = m + h * perp;                % Joint point Ji

        % Plot links
        plot([B(i,1) J(1)], [B(i,2) J(2)], 'r-', 'LineWidth', 2); % Link 1
        plot([J(1) P(i,1)], [J(2) P(i,2)], 'g-', 'LineWidth', 2); % Link 2

        % Mark joints
        plot(B(i,1), B(i,2), 'ko', 'MarkerFaceColor', 'k');
        plot(J(1), J(2), 'ro', 'MarkerFaceColor', 'r');
        plot(P(i,1), P(i,2), 'bo', 'MarkerFaceColor', 'b');
    end

    title('Rigid 3-3R Planar Parallel Manipulator');
    drawnow;
end

% Save animation
obj = VideoWriter('animation.avi');
obj.Quality = 100;
obj.FrameRate = 20;
open(obj);
