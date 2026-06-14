function robot_obj = ur10robot()
    robot_obj = generate_ur10_model();
    robot_obj.base.t = [1; 1; 0];
end

function sys = generate_ur10_model()
    deg2r = pi/180;
    
    % Kinematic Parameters (Lengths in meters)
    link_len = [0; -0.612; -0.5723; 0; 0; 0];
    link_off = [0.1273; 0; 0; 0.163941; 0.1157; 0.0922];
    link_tws = [1.570796327; 0; 0; 1.570796327; -1.570796327; 0];
    j_angles = zeros(6,1);
    
    % Denavit-Hartenberg Table
    DH_matrix = [j_angles, link_off, link_len, link_tws];
    
    % Mass and Inertia parameters
    m_links = [7.1, 12.7, 4.27, 2.000, 2.000, 0.365];

    CoM_matrix = [
        0.021, 0, 0.027;
        0.38,  0, 0.158;
        0.24,  0, 0.068;
        0.0,   0.007, 0.018;
        0.0,   0.007, 0.018;
        0,     0, -0.026];    

    sys = SerialLink(DH_matrix, 'name', 'UR10_Arm', 'manufacturer', 'Universal Robotics');
    
    % Assign Mass Data
    L = sys.links;
    for k = 1:6
        L(k).m = m_links(k);
        L(k).r = CoM_matrix(k,:);
    end

    % Workspace variable injections
    if nargin == 0
        assignin('caller', 'ur10', sys);
        assignin('caller', 'qz', [0 0 0 0 0 0]); 
        assignin('caller', 'qr', [180 0 0 0 90 0] * deg2r); 
    end
end