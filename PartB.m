% UR10 Inverse Kinematics and Control Script
clc;
clear;
close all;

% Initialize manipulator
arm_ur10 = ur10robot();
motion_duration = 5;

load('partA_H_trajectory.mat', 'H_traj');
load('partA_D_trajectory.mat', 'D_traj');

samples = size(H_traj, 3);
delta_t = motion_duration / (samples - 1);
fprintf('Sampling rate at: %.2f\n', delta_t);
t_array = linspace(0, motion_duration, samples);

% Joint variable initialization
q_init = [-1.7752; -1.1823; 0.9674; 0.2149; 1.3664; 1.5708];
theta_path = zeros(samples, 6);
omega_path = zeros(samples, 6);

% Hand-eye calibration matrix
Trans_he = [0 0 -1 0.1;
            0 1  0 0.1;
            1 0  0 0.0;
            0 0  0 1.0];

current_q = q_init;
theta_path(1, :) = current_q';

% Compute Inverse Kinematics iteratively
for step = 2:samples
    Target_H = H_traj(:, :, step);      
    Target_e = Target_H * Trans_he;          

    Current_e = arm_ur10.fkine(current_q);      
    PoseError = tr2delta(Current_e.T, Target_e);

    Jac = arm_ur10.jacobe(current_q);
    q_velocity = pinv(Jac) * (PoseError / delta_t);

    current_q = current_q + q_velocity * delta_t;
    theta_path(step, :) = current_q';
    omega_path(step, :) = q_velocity';
end

width_door = 1.0;
height_door = 2.0;

% Set up animation environment
figure('Name','UR10 Kinematics Simulation');
axis([-1 2 0 3 0 2.5]);
view(45, 30); grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

trplot(eye(4), 'length', 0.2, 'color','k');
plot3([-1 2], [0 0], [0 0], 'k--');
plot3([0 0], [-0.5 3], [0 0], 'k--');

arm_ur10.plot(theta_path(1, :), 'workspace', [-1 2 -0.5 3 0 1.5], 'view', [45 30]);

anim_step = 4;
plot_indices = 1:anim_step:samples;
if plot_indices(end) ~= samples
    plot_indices = [plot_indices, samples];
end

for idx = 1:length(plot_indices)
    curr_idx = plot_indices(idx);
    
    arm_ur10.animate(theta_path(curr_idx, :));  
    
    Frame_EE = arm_ur10.fkine(theta_path(curr_idx, :));   
    Frame_H = H_traj(:, :, curr_idx);         
    Frame_D = D_traj(:, :, curr_idx);         

    if exist('door_patch','var'), delete(door_patch); end
    if exist('ax_hinge','var'), delete(ax_hinge); end
    if exist('plt_e','var'), delete(plt_e); end
    if exist('plt_H','var'), delete(plt_H); end
    if exist('plt_D','var'), delete(plt_D); end

    plt_D = trplot(Frame_D, 'frame','D','color','b','length',0.5);
    plt_e = trplot(Frame_EE.T, 'frame','e','color','g','length',0.5);
    plt_H = trplot(Frame_H, 'frame','H','color','r','length',0.5);

    door_pts = [0 0 0; width_door 0 0; width_door 0 height_door; 0 0 height_door; 0 0 0]';
    door_w_coords = Frame_D(1:3,1:3) * door_pts + Frame_D(1:3,4);
    door_patch = plot3(door_w_coords(1,:), door_w_coords(2,:), door_w_coords(3,:), 'Color', [.8 .4 .1], 'LineWidth', 2);
    ax_hinge = plot3([0 0], [2 0], [0 0], 'k--', 'LineWidth', 1);

    title(sprintf('Animation Time: %.2f seconds', t_array(curr_idx)));
    drawnow;
    
    if idx < length(plot_indices)
        pause((plot_indices(idx+1) - curr_idx) * delta_t);
    end
end

% ------------------------------------------------------------------------
% Joint Dynamics Plots
% ------------------------------------------------------------------------
figure('Name','Joint Kinematics Profiles');
plot(t_array, theta_path, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Joint Angles [rad]');
legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','Location','best');
grid on; title('UR10 Joint Angles vs. Time');

figure('Name','Joint Velocity Profiles');
plot(t_array, omega_path, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Joint Velocities [rad/s]');
legend('\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6','Location','best');
grid on; title('UR10 Joint Velocity Profiles');

% ------------------------------------------------------------------------
% End-Effector Absolute Trajectory
% ------------------------------------------------------------------------
pos_EE_path = zeros(samples, 3);
quat_EE_path = zeros(samples, 4);

for k = 1:samples
    g_e = arm_ur10.fkine(theta_path(k, :));
    pos_EE_path(k, :) = transl(g_e)';
    quat_EE_path(k, :) = UnitQuaternion(g_e).double;
end

figure('Name','End-Effector Absolute Path');
plot3(pos_EE_path(:,1), pos_EE_path(:,2), pos_EE_path(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Path of End-Effector (World Frame)');

figure('Name','End-Effector Absolute Orientation');
plot(t_array, quat_EE_path, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3','Location','best');
grid on; title('End-Effector Quaternion vs. Time');

% ------------------------------------------------------------------------
% End-Effector Relative Trajectory (w.r.t Handle)
% ------------------------------------------------------------------------
rel_pos_path = zeros(samples, 3);
rel_quat_path = zeros(samples, 4);

for k = 1:samples
    g_H = H_traj(:, :, k);
    g_e = arm_ur10.fkine(theta_path(k, :));
    g_diff = inv(g_H) * g_e.T;
    rel_pos_path(k, :) = transl(g_diff)';
    rel_quat_path(k, :) = UnitQuaternion(g_diff).double;
end

figure('Name','Relative Position to Handle');
plot3(rel_pos_path(:,1), rel_pos_path(:,2), rel_pos_path(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Relative 3D Position: End-Effector w.r.t. Handle');

figure('Name','Relative Orientation to Handle');
plot(t_array, rel_quat_path, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3','Location','best');
grid on; title('Relative Quaternion: End-Effector w.r.t. Handle');