function xdot = drone_controller(q_m, l, g, k_f, hard_tilt, gains, trajectory, drone_configuration, t, x)

    %% moment of inertia
    Jx= 1.395 * 10^-5 ; % moment of inertia about  X axis  (kg*m^2)
    Jy= 1.436 * 10^-5 ; % moment of inertia about  Y axis  (kg*m^2)
    Jz= 2.173 * 10^-5 ; % moment of inertia about  Z axis  (kg*m^2)
    Jr = 3.2e-7; % Polar moment of inertia of the propeller–motor assembly (kg*m^2)
    J1 =  (Jy - Jz)/Jx; 
    J2 =  (Jz - Jx)/Jy;
    J3 =  (Jx - Jy)/Jz;
    g_u = x(13) - x(14) + x(15) - x(16); % motor speed difference

    %% Trajectory
    % pd = [2*sin(0.5*t); 2*cos(0.5*t); 0.1*t]; % desired trajectory
    % vd = [1*cos(0.5*t); -1*sin(0.5*t); 0.1]; % desired velocity

    [pd, vd] = ref_trajectory(t, trajectory);   % or 'hoverZ', 'spiral'
  
    %% Controller
    % controller gains
    kp_phi = 10; kp_theta = 10; kp_psi = 10; 
    kd_phi = 6 ; kd_theta = 6 ; kd_psi = 6 ; 
    
    kp_z = 10;  % inner loop proportional gains
    kd_z = 6; % inner loop derivative gains

    % kp_x = 1.2; kp_y = 1.2; 
    % kd_x = 1.5; kd_y = 1.5;
    % ki_x = 0.31; ki_y = 0.31;
    
    % kp_x = 1.8; kp_y = 1.8; 
    % kd_x = 2.5; kd_y = 2.5;
    % ki_x =0.31; ki_y = 0.31;
    k_a_ff = 1;  

    kp_x = gains.kp_x;   kd_x = gains.kd_x;   ki_x = gains.ki_x;
    kp_y = gains.kp_y;   kd_y = gains.kd_y;   ki_y = gains.ki_y;
    %% Position controller

    % Altitute Controller
    uz_d = kp_z*(pd(3,1)-x(5))+ kd_z*(vd(3)-x(6));
    uz = g;
    F = q_m*(uz + uz_d)/((cos(x(9))*cos(x(7))));

    % translational controller
    a_x_ff = -0.5*sin(0.5*t); % feedforward acceleration in x direction
    a_y_ff = -0.5*cos(0.5*t);

    ux_d = kp_x*(pd(1)-x(1)) + kd_x*(vd(1)-x(2)) + ki_x*x(17) + k_a_ff*a_x_ff;
    uy_d = kp_y*(pd(2)-x(3)) + kd_y*(vd(2)-x(4)) + ki_y*x(18) + k_a_ff*a_y_ff;

    % angle controller
    phi_d = (ux_d*sin(x(11)) - uy_d*cos(x(11)))/g;
    theta_d = (ux_d*cos(x(11)) + uy_d*sin(x(11)))/g;

    if hard_tilt == 1  % hard tilt mode
        phi_d = max(min(phi_d, deg2rad(30)), -deg2rad(30));
        theta_d = max(min(theta_d, deg2rad(30)), -deg2rad(30));
    end
    %% Atttitute controller

    uPhi = x(12)*x(10)*J1;
    uTheta = x(12)*x(8)*J2;
    uPsi = x(8)*x(10)*J3;

    phi_dot_d = 0;
    theta_dot_d = 0;
    psi_dot_d = 0;
    psi_d = 0;

    uPhi_d = kp_phi*(phi_d - x(7)) +  kd_phi*(phi_dot_d - x(8));
    uTheta_d = kp_theta*(theta_d - x(9)) + kd_theta*(theta_dot_d - x(10));
    uPsi_d = kp_psi*(psi_d - x(11)) +  kd_psi*(psi_dot_d - x(12));

    tau_phi = (uPhi + uPhi_d)*(Jx/l);
    tau_theta = (uTheta + uTheta_d)*(Jy/l);
    tau_psi = (uPsi + uPsi_d)*(Jz);

    %% Desired Motor Speed
    k_m = 7.2385e-10;
    k = k_m/k_f;
    to = 20 ; % motor gain (s^-1)

    l_x = l/sqrt(2);

    switch drone_configuration
        case 'plus'
            drone_config = [1 1 1 1;
                   0 l 0 -l;
                   -l 0 l 0;
                   k -k k -k]; % for plus configuration
        case 'cross'
            drone_config = [1 1 1 1;
                   -l_x l_x l_x -l_x;
                   -l_x -l_x l_x l_x;
                   k -k k -k]; % for x(cross) configuration
        otherwise
            error('Unknown drone configuration');
    end

    Wd_s = (1/k_f) * drone_config^-1 * [F; tau_phi; tau_theta; tau_psi];

    W_d = sqrt(Wd_s); % Desired Motor Speed
    W_d = min(max(W_d, 0), 3000); % Limit motor speed to [0, 3000] rad/s


    %% Motor Inputs (radian/s)

    f = zeros(4,1);
    m = zeros(4,1);

    for i = 1 : 4
        f(i) = k_f*x(12 + i)^2;
        m(i) = k_m*x(12 + i)^2;
    end

    switch drone_configuration
        case 'plus'
            u1 = f(1) + f(2) + f(3) + f(4);
            u2 = l*(f(2) - f(4));
            u3 = l*(f(3) - f(1));
            u4 = m(1) - m(2) + m(3) - m(4);
            
        case 'cross'
            u1 = f(1) + f(2) + f(3) + f(4);
            u2 = l*(f(2) - f(4));
            u3 = l*(f(3) - f(1));
            u4 = m(1) - m(2) + m(3) - m(4);

        otherwise
            error('Unknown drone configuration');
    end

    %% 6DOF Equations and Motor Dynamic Model

    xdot(1) = x(2);
    xdot(2) = ((cos(x(7))*sin(x(9))*cos(x(11)) + sin(x(7))*sin(x(11)))*u1/q_m);
    xdot(3) = x(4);
    xdot(4) = ((cos(x(7))*sin(x(9))*sin(x(11)) - sin(x(7))*cos(x(11)))*u1/q_m);
    xdot(5) = x(6);
    xdot(6) = -g + (cos(x(7))*cos(x(9)))* u1/q_m;
    xdot(7) = x(8);
    xdot(8) = x(12)*x(10)*J1 - Jr*x(10)*g_u/Jx + l*u2/Jx;
    xdot(9) = x(10);
    xdot(10)= x(12)*x(8)*J2 + Jr*x(8)*g_u/Jy + l*u3/Jy;
    xdot(11) = x(12);
    xdot(12) = x(8)*x(10)*J3 + l*u4/Jz;
    %% Motor Dynamics
    xdot(13) = to*(W_d(1) - x(13));
    xdot(14) = to*(W_d(2) - x(14));
    xdot(15) = to*(W_d(3) - x(15));
    xdot(16) = to*(W_d(4) - x(16));

    xdot(17) = pd(1) - x(1); % Position error in x
    xdot(18) = pd(2) - x(3); %

    xdot = xdot';

end
