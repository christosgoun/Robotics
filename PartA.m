% Trajectory Generation Script
clc;
clear;
close all;

% Physical dimensions of the setup
dist_knob = 0.1;         
height_knob = 0.7;          
width_door = 1.0;          
height_door = 2.0; 

time_phase1 = 2; 
time_phase2 = 3;  
t_total = time_phase1 + time_phase2;
steps_p1 = 201; 
steps_p2 = 300;

% Generating motion arrays using column vectors
angle_twist = [tpoly(0, deg2rad(-45), steps_p1); tpoly(deg2rad(-45), 0, steps_p2)];   
angle_swing = [zeros(steps_p1, 1); tpoly(0, deg2rad(-30), steps_p2)]; 

num_samples = length(angle_twist);
t_vec = linspace(0, t_total, num_samples);
dt_step = t_total / (num_samples - 1);
fprintf('Execution Sampling Rate: %.2f s\n', dt_step);

% Data allocation
Handle_Frames = zeros(4, 4, num_samples);
Door_Frames = zeros(4, 4, num_samples);

Hinge_Origin = transl([0, 2, 0]);   

figure('Name', 'System Kinematics', 'Color', 'white');
for i = 1:num_samples
    % Forward kinematics for the door and handle
    Frame_D0 = Hinge_Origin * trotz(angle_swing(i));            
    Frame_HD = transl([width_door - dist_knob, 0, height_knob]) * trotz(-pi/2) * trotx(angle_twist(i));
    Frame_H0 = Frame_D0 * Frame_HD;                         

    Door_Frames(:,:,i) = Frame_D0;
    Handle_Frames(:,:,i) = Frame_H0;

    % Visualization updates
    clf; axis equal; grid on; view(35,25); hold on;
    xlabel('X-axis (m)'); ylabel('Y-axis (m)'); zlabel('Z-axis (m)');

    trplot(eye(4), 'frame', 'W', 'color', 'k', 'length', 0.5);
    trplot(Frame_D0, 'frame', 'D', 'color', 'b', 'length', 0.5);
    trplot(Frame_H0, 'frame', 'H', 'color', 'r', 'length', 0.5);

    door_geom = [0 0 0; width_door 0 0; width_door 0 height_door; 0 0 height_door; 0 0 0]';               
    door_world = Frame_D0(1:3,1:3) * door_geom + Frame_D0(1:3,4);
    plot3(door_world(1,:), door_world(2,:), door_world(3,:), 'Color', [0.8 0.4 0.1], 'LineWidth', 2);
    plot3([0 0], [2 0], [0 0], 'k--', 'LineWidth', 1); 

    axis([-0.2 1.4 0 2.8 0 2.2]);
    title(sprintf('Simulation Time: %.2f sec', t_vec(i)));
    drawnow; 
    pause(dt_step);
end

% Ensure file names match exactly what Part B expects
H_traj = Handle_Frames;
D_traj = Door_Frames;
save('partA_H_trajectory.mat', 'H_traj');
save('partA_D_trajectory.mat', 'D_traj');

% --- Additional Plots ---

% Extract handle position and quaternion
pos_h_path = zeros(num_samples, 3);   
quat_h_path = zeros(num_samples, 4);   

for i = 1:num_samples
    gH = Handle_Frames(:,:,i);
    pos_h_path(i, :) = transl(gH)';                     
    quat_h_path(i, :) = UnitQuaternion(gH).double;       
end

figure('Name','Handle Position Tracking','Color','w');
plot(t_vec, pos_h_path(:,1), 'LineWidth', 1.5); hold on;
plot(t_vec, pos_h_path(:,2), 'LineWidth', 1.5);
plot(t_vec, pos_h_path(:,3), 'LineWidth', 1.5);
grid on; axis tight;
xlabel('Time [s]'); ylabel('Position [m]');
legend('X_h', 'Y_h', 'Z_h', 'Location', 'best');
title('Handle Position Components Over Time');

figure('Name','Handle Orientation Tracking','Color','w');
plot(t_vec, quat_h_path(:,1), 'LineWidth', 1.5); hold on;
plot(t_vec, quat_h_path(:,2), 'LineWidth', 1.5);
plot(t_vec, quat_h_path(:,3), 'LineWidth', 1.5);
plot(t_vec, quat_h_path(:,4), 'LineWidth', 1.5);
grid on; axis tight;
xlabel('Time [s]'); ylabel('Quaternion Components');
legend('q_0', 'q_1', 'q_2', 'q_3', 'Location', 'best');
title('Handle Orientation (Unit Quaternion) Over Time');

figure('Name','Motion Profiles','Color','w');
yyaxis left
plot(t_vec, rad2deg(angle_twist), 'LineWidth', 1.5);
ylabel('Twist Angle [deg]');
yyaxis right
plot(t_vec, rad2deg(angle_swing), '--', 'LineWidth', 1.5);
ylabel('Swing Angle [deg]');
grid on; axis tight;
xlabel('Time [s]');
legend('Knob Twist', 'Door Swing', 'Location', 'best');
title('Kinematic Motion Profiles');