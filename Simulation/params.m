P.gravity = 9.8;

% set_param('mavsim_chap6', 'DecoupleContODEIntegFromDiscRates', 'on')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 20;
P.Jx   = .876;
P.Jy   = .876;
P.Jz   = .0667;
P.Jxz  = 0;
P.Jball = .876;

P.R_ball = .12;
P.R_cm = .24;	% center of ball to CM of body;
P.k_m = 1;		% Motor force per current; (k_m * delta motor = Force)

% Maximum Angles
P.delta_A_max = 45 * pi/180;
P.delta_B_max = 45 * pi/180;
P.delta_C_max = 45 * pi/180;
P.e_phi_max   = 45 * pi/180;
P.e_theta_max = 45 * pi/180;
P.e_beta_max  = 45 * pi/180;

P.rollMax = pi/2;
P.pitchMax = pi/2;

% Sensor Parameters:
P.bias_gyro_x = 0;
P.bias_gyro_y = 0;
P.bias_gyro_z = 0;

% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 0;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;
P.Ts_gps = 1;
P.Ts_sensors = P.Ts*1;
P.tau = 5; % ~ 100 Hz 

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

% run trim commands
% [x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
% P.u_trim = u_trim;
% P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
% P.pn0    = 0;  % initial North position
% P.pe0    = 0;  % initial East position
% P.pd0    = 0;  % initial Down position (negative altitude)
% P.u0     = x_trim(4);  % initial velocity along body x-axis
% P.v0     = x_trim(5);  % initial velocity along body y-axis
% P.w0     = x_trim(6);  % initial velocity along body z-axis
% P.phi0   = x_trim(7);  % initial roll angle
% P.theta0 = x_trim(8);  % initial pitch angle
% P.psi0   = x_trim(9);  % initial yaw angle
% P.p0     = x_trim(10);  % initial body frame roll rate
% P.q0     = x_trim(11);  % initial body frame pitch rate
% P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
% [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
%     = compute_tf_model(x_trim,u_trim,P);
% compute_tf_model;

% linearize the equations of motion around trim conditions
% [A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% Autopilot Zones
% P.altitude_take_off_zone = 100;
% P.altitude_hold_zone = 50;
% P.takeoff_angle = 20 *pi/180;
% 
% calculate_gains;
